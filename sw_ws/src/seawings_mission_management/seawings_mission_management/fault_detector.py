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
                ('gps_timeout', 10.0), # Timeout for GPS data
                ('min_satellites', 6),
                ('min_fix_type', 3),  # 3D fix
                ('check_interval', 2.0),
                ('estimator_timeout', 10.0),
            ]
        )

        # Get parameters
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.min_satellites = self.get_parameter('min_satellites').value
        self.min_fix_type = self.get_parameter('min_fix_type').value
        self.check_interval = self.get_parameter('check_interval').value
        self.estimator_timeout = self.get_parameter('estimator_timeout').value

        # Log parameters
        self.get_logger().info(f'Parameters: gps_timeout={self.gps_timeout}s, '
                               f'min_satellites={self.min_satellites}, '
                               f'min_fix_type={self.min_fix_type}, '
                               f'check_interval={self.check_interval}s, '
                               f'estimator_timeout={self.estimator_timeout}s')

        # State variables
        self.gps_status = None
        self.estimator_status = None
        self.vehicle_status = None
        self.last_gps_time = None
        self.last_estimator_time = None
        self.emergency_triggered = False
        self.data_lock = Lock()
        self.last_command_result = None

        # Define QoS profile for PX4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher for direct PX4 control (no MAVROS!)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos_profile
        )

        # Subscribers with PX4 QoS compatibility
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

        self.get_logger().info('FaultDetector node initialized with direct PX4 control (no MAVROS!)')

    def gps_callback(self, msg):
        with self.data_lock:
            self.gps_status = msg
            if msg.fix_type >= self.min_fix_type:
                self.last_gps_time = time.time()

    def estimator_callback(self, msg):
        with self.data_lock:
            self.estimator_status = msg
            self.last_estimator_time = time.time()

    def status_callback(self, msg):
        with self.data_lock:
            self.vehicle_status = msg

    def command_ack_callback(self, msg):
        with self.data_lock:
            self.last_command_result = msg
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info('✅ Emergency landing mode accepted by PX4')
                else:
                    self.get_logger().error(f'❌ Emergency landing mode failed: result={msg.result}')
                    self.emergency_triggered = False  # Allow retry

    def get_current_mode(self):
        """Get current flight mode"""
        if not self.vehicle_status:
            return "UNKNOWN"
        
        mode_map = {
            0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "AUTO.LOITER",
            4: "AUTO.MISSION", 5: "AUTO.RTL", 6: "AUTO.LAND", 7: "AUTO.TAKEOFF",
            8: "OFFBOARD", 9: "STABILIZED", 10: "ACRO"
        }
        
        return mode_map.get(self.vehicle_status.nav_state, f"UNKNOWN({self.vehicle_status.nav_state})")

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send command directly to PX4"""
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
        if self.emergency_triggered:
            return

        with self.data_lock:
            current_time = time.time()

            # Check GPS health
            gps_healthy = self.check_gps_health(current_time)

            # Check estimator health
            estimator_healthy = self.check_estimator_health(current_time)

            # Log system status
            gps_status_str = "HEALTHY" if gps_healthy else "DEGRADED"
            est_status_str = "HEALTHY" if estimator_healthy else "DEGRADED"
            mode = self.get_current_mode()

            self.get_logger().info(f'System Health - GPS: {gps_status_str}, Estimator: {est_status_str}, Mode: {mode}')

            # Trigger emergency landing if critical systems are compromised
            if not gps_healthy:
                if self.gps_status is None:
                    self.trigger_emergency_landing("No GPS data received")
                elif self.gps_status.fix_type < 2:
                    self.trigger_emergency_landing(f"GPS fix lost (fix_type={self.gps_status.fix_type})")
                elif self.gps_status.satellites_used < self.min_satellites:
                    self.trigger_emergency_landing(f"Low satellite count ({self.gps_status.satellites_used})")
                elif self.last_gps_time is None or (current_time - self.last_gps_time > self.gps_timeout):
                    self.trigger_emergency_landing("GPS data timeout")
                else:
                    self.get_logger().warn("GPS unhealthy but not critical for immediate landing.")

            if not estimator_healthy:
                if self.estimator_status is None:
                    self.trigger_emergency_landing("No Estimator Status data received")
                elif self.last_estimator_time is None or (current_time - self.last_estimator_time > self.estimator_timeout):
                    self.trigger_emergency_landing("Estimator Status data timeout")
                else:
                    self.trigger_emergency_landing("Estimator flags indicate degraded position estimate")

    def check_gps_health(self, current_time):
        """Check GPS health status"""
        if self.gps_status is None:
            return False

        if self.gps_status.fix_type < self.min_fix_type:
            return False

        if self.gps_status.satellites_used < self.min_satellites:
            return False

        if self.last_gps_time is None or (current_time - self.last_gps_time > self.gps_timeout):
            return False

        if hasattr(self.gps_status, 'jamming_state') and self.gps_status.jamming_state == 3:
            return False
        
        if hasattr(self.gps_status, 'spoofing_state') and self.gps_status.spoofing_state >= 2:
            return False

        return True

    def check_estimator_health(self, current_time):
        """Check estimator health status"""
        if self.estimator_status is None:
            return False

        if self.last_estimator_time is None or (current_time - self.last_estimator_time > self.estimator_timeout):
            return False

        if hasattr(self.estimator_status, 'gps_check_fail_flags'):
            gps_flags = self.estimator_status.gps_check_fail_flags
            if gps_flags & (1 << 0):  # GPS_CHECK_FAIL_GPS_FIX
                return False
            if gps_flags & (1 << 1):  # GPS_CHECK_FAIL_MIN_SAT_COUNT
                return False
            if gps_flags & (1 << 3) or gps_flags & (1 << 4):  # HORZ_ERR or VERT_ERR
                return False

        if hasattr(self.estimator_status, 'control_mode_flags'):
            control_flags = self.estimator_status.control_mode_flags
            gps_active = control_flags & (1 << 2)  # CS_GPS
            if not gps_active:
                return False

        return True

    def trigger_emergency_landing(self, reason):
        """Trigger emergency landing"""
        if self.emergency_triggered:
            return

        self.emergency_triggered = True
        self.get_logger().error(f'EMERGENCY LANDING TRIGGERED: {reason}')

        # Send LAND mode command directly to PX4
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # base mode (1 = custom mode)
            param2=6.0   # custom mode (6 = LAND for PX4)
        )
        
        self.get_logger().info('📡 Emergency landing command sent directly to PX4')

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