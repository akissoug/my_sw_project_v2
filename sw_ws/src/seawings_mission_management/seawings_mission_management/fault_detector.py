#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, SensorGps, VehicleStatus, VehicleCommand, VehicleCommandAck
import time
from threading import Lock

class FaultDetector(Node):
    def __init__(self):
        super().__init__('fault_detector')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gps_timeout', 20.0),  
                ('min_satellites', 6),
                ('min_fix_type', 3),
                ('check_interval', 5.0),  # INCREASED from 2.0
                ('estimator_timeout', 20.0),  
                ('startup_grace_period', 15.0),  
                ('sitl_mode', True),
                ('emergency_command_cooldown', 5.0),  
                ('gps_failure_count_threshold', 3),  
            ]
        )

        # Get parameters
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.min_satellites = self.get_parameter('min_satellites').value
        self.min_fix_type = self.get_parameter('min_fix_type').value
        self.check_interval = self.get_parameter('check_interval').value
        self.estimator_timeout = self.get_parameter('estimator_timeout').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        self.emergency_command_cooldown = self.get_parameter('emergency_command_cooldown').value  # NEW
        self.gps_failure_count_threshold = self.get_parameter('gps_failure_count_threshold').value  # NEW

        # Log parameters
        self.get_logger().info(f'Parameters: gps_timeout={self.gps_timeout}s, '
                               f'min_satellites={self.min_satellites}, '
                               f'min_fix_type={self.min_fix_type}, '
                               f'check_interval={self.check_interval}s, '
                               f'estimator_timeout={self.estimator_timeout}s, '
                               f'emergency_cooldown={self.emergency_command_cooldown}s')

        # State variables
        self.gps_status = None
        self.estimator_status = None
        self.vehicle_status = None
        self.last_gps_time = None
        self.last_estimator_time = None
        self.emergency_triggered = False
        self.data_lock = Lock()
        self.last_command_result = None
        self.node_start_time = time.time()
        self.estimator_ever_received = False
        
        # NEW: Rate limiting and GPS failure tracking
        self.last_emergency_command_time = 0
        self.emergency_triggered_time = None
        self.gps_degraded_count = 0
        self.consecutive_gps_failures = 0

        # Define QoS profile for PX4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher for direct PX4 control
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos_profile
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile=px4_qos_profile
        )

        self.estimator_sub = self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.estimator_callback,
            qos_profile=px4_qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile=px4_qos_profile
        )

        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            qos_profile=px4_qos_profile
        )

        # Timer for periodic health checks
        self.timer = self.create_timer(self.check_interval, self.check_system_health)

        self.get_logger().info('FaultDetector node initialized...')
        if self.sitl_mode:
            self.get_logger().info(f'🎮 SITL mode active with {self.startup_grace}s startup grace period')

    def gps_callback(self, msg):
        with self.data_lock:
            self.gps_status = msg
            if msg.fix_type >= self.min_fix_type:
                self.last_gps_time = time.time()
                self.consecutive_gps_failures = 0  # Reset on good GPS

    def estimator_callback(self, msg):
        with self.data_lock:
            self.estimator_status = msg
            self.last_estimator_time = time.time()
            self.estimator_ever_received = True

    def status_callback(self, msg):
        with self.data_lock:
            self.vehicle_status = msg

    def command_ack_callback(self, msg):
        with self.data_lock:
            self.last_command_result = msg
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info('✅ Emergency landing mode accepted by PX4')
                    self.emergency_triggered = True  # Confirm it worked
                else:
                    self.get_logger().error(f'❌ Emergency landing mode failed: result={msg.result}')
                    # Don't reset emergency_triggered here - let cooldown handle it

    def is_in_startup_grace_period(self):
        """Check if still in startup grace period"""
        return (time.time() - self.node_start_time) < self.startup_grace

    def get_current_mode(self):
        if not self.vehicle_status:
            return "UNKNOWN"
        
        mode_map = {
            0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "AUTO.LOITER",
            4: "AUTO.MISSION", 5: "AUTO.RTL", 6: "AUTO.LAND", 7: "AUTO.TAKEOFF",
            8: "OFFBOARD", 9: "STABILIZED", 10: "ACRO", 11: "AUTO.LAND_ENGAGED",
            12: "AUTO.PRECLAND", 13: "ORBIT", 14: "AUTO.VTOL_TAKEOFF",
            15: "EXTERNAL1", 16: "EXTERNAL2", 17: "EXTERNAL3",
            18: "EXTERNAL4", 19: "EXTERNAL5", 20: "EXTERNAL6",
            21: "EXTERNAL7", 22: "EXTERNAL8"
        }
        
        return mode_map.get(self.vehicle_status.nav_state, f"UNKNOWN({self.vehicle_status.nav_state})")

    def is_failsafe_active(self):
        """Check if PX4 failsafe is already active"""
        if self.vehicle_status:
            return self.vehicle_status.failsafe
        return False
    
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)

    def check_system_health(self):
        """Main system health monitoring logic"""
        # Skip if already handling emergency
        current_mode = self.get_current_mode()
        if current_mode in ['AUTO.LAND', 'AUTO.LAND_ENGAGED']:
            self.get_logger().debug('Already in landing mode - skipping checks')
            return

        # Skip checks during startup grace period
        if self.is_in_startup_grace_period():
            self.get_logger().debug(f'In startup grace period ({time.time() - self.node_start_time:.1f}s)')
            return
        
        if self.is_failsafe_active():
            self.get_logger().info('PX4 failsafe active - letting it handle the situation')
            return

        with self.data_lock:
            current_time = time.time()

            # Check GPS health with tolerance
            gps_healthy = self.check_gps_health_tolerant(current_time)

            # Check estimator health (more lenient in SITL)
            estimator_healthy = self.check_estimator_health(current_time)

            # Log system status
            gps_status_str = "HEALTHY" if gps_healthy else "DEGRADED"
            est_status_str = "HEALTHY" if estimator_healthy else "DEGRADED"

            self.get_logger().info(f'System Health - GPS: {gps_status_str}, Estimator: {est_status_str}, Mode: {current_mode}')

            # Only trigger emergency after multiple consecutive failures
            if not gps_healthy:
                self.consecutive_gps_failures += 1
                
                if self.consecutive_gps_failures >= self.gps_failure_count_threshold:
                    # Determine specific reason
                    if self.gps_status is None:
                        reason = "No GPS data received"
                    elif self.gps_status.fix_type < 2:
                        reason = f"GPS fix lost (fix_type={self.gps_status.fix_type})"
                    elif self.gps_status.satellites_used < self.min_satellites:
                        reason = f"Low satellite count ({self.gps_status.satellites_used})"
                    else:
                        reason = "GPS data timeout"
                    
                    self.trigger_emergency_landing_with_cooldown(reason)
                else:
                    self.get_logger().warn(f'GPS degraded ({self.consecutive_gps_failures}/{self.gps_failure_count_threshold} failures)')

    def check_gps_health_tolerant(self, current_time):
        """Check GPS health with tolerance for slow systems"""
        if self.gps_status is None:
            return False

        # Accept 2D fix on slow systems temporarily
        if self.gps_status.fix_type >= 2:
            # Update time even for 2D fix
            if self.gps_status.fix_type == 2:
                self.last_gps_time = current_time
                self.get_logger().debug('Accepting 2D GPS fix on slow system')

        if self.gps_status.fix_type < self.min_fix_type:
            # Don't immediately fail
            if hasattr(self, 'gps_degraded_count'):
                self.gps_degraded_count += 1
            else:
                self.gps_degraded_count = 1
            
            # More tolerance
            if self.gps_degraded_count < 5:
                return True
            return False
        else:
            self.gps_degraded_count = 0

        # Be more tolerant with satellite count
        if self.gps_status.satellites_used < self.min_satellites:
            if self.gps_status.satellites_used >= 5:
                self.get_logger().debug(f'Accepting {self.gps_status.satellites_used} satellites (minimum is {self.min_satellites})')
                return True
            return False

        # Double timeout for slow systems
        adjusted_timeout = self.gps_timeout * 2 if self.sitl_mode else self.gps_timeout
        if self.last_gps_time is None or (current_time - self.last_gps_time > adjusted_timeout):
            return False

        # Check jamming/spoofing
        if hasattr(self.gps_status, 'jamming_state') and self.gps_status.jamming_state == 3:
            return False
        
        if hasattr(self.gps_status, 'spoofing_state') and self.gps_status.spoofing_state >= 2:
            return False

        return True

    def check_estimator_health(self, current_time):
        """Check estimator health status - more lenient in SITL"""
        if self.sitl_mode and not self.estimator_ever_received:
            return True
            
        if self.estimator_status is None:
            return False

        timeout = self.estimator_timeout * 2 if self.sitl_mode else self.estimator_timeout
        if self.last_estimator_time is None or (current_time - self.last_estimator_time > timeout):
            return False

        if hasattr(self.estimator_status, 'gps_check_fail_flags'):
            gps_flags = self.estimator_status.gps_check_fail_flags
            if gps_flags & (1 << 0):
                return False

        return True

    def trigger_emergency_landing_with_cooldown(self, reason):
        """Trigger emergency landing with rate limiting"""
        current_time = time.time()
        
        # Check if we're in cooldown
        if current_time - self.last_emergency_command_time < self.emergency_command_cooldown:
            self.get_logger().debug(f'Emergency command in cooldown ({self.emergency_command_cooldown:.0f}s)')
            return
        
        # Check if already triggered and hasn't timed out
        if self.emergency_triggered:
            if self.emergency_triggered_time and (current_time - self.emergency_triggered_time) > 30.0:
                # Reset after 30 seconds if no success
                self.emergency_triggered = False
                self.emergency_triggered_time = None
                self.get_logger().info('Resetting emergency state after timeout')
            else:
                return

        # Check current mode again
        current_mode = self.get_current_mode()
        if current_mode in ['AUTO.LAND', 'AUTO.LAND_ENGAGED']:
            self.get_logger().info('Already in landing mode')
            return

        self.emergency_triggered = True
        self.emergency_triggered_time = current_time
        self.last_emergency_command_time = current_time
        
        self.get_logger().error(f'EMERGENCY LANDING TRIGGERED: {reason}')

        # Send LAND mode command
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        
        self.get_logger().info(f'📡 Emergency command sent - next allowed in {self.emergency_command_cooldown}s')

def main(args=None):
    rclpy.init(args=args)
    node = FaultDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down FaultDetector...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
