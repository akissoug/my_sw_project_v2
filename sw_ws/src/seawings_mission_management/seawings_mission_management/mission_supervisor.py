#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus, VehicleCommand, VehicleCommandAck
from std_msgs.msg import String
import time
from threading import Lock
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    ARMED = 1
    TAKEOFF = 2
    MISSION = 3
    RTL = 4
    LANDING = 5
    EMERGENCY = 6
    DISARMED = 7

class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        
        # Declare parameters
        self.declare_parameter('heartbeat_timeout', 30.0)
        self.declare_parameter('telemetry_timeout', 10.0)
        self.declare_parameter('mission_timeout', 3600.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('critical_battery_threshold', 5.0)
        self.declare_parameter('startup_grace_period', 10.0)
        self.declare_parameter('command_cooldown', 10.0)  
        
        # Get parameters
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.telemetry_timeout = self.get_parameter('telemetry_timeout').value
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.check_interval = self.get_parameter('check_interval').value
        self.critical_battery = self.get_parameter('critical_battery_threshold').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        self.command_cooldown = self.get_parameter('command_cooldown').value  
        
        # State variables
        self.mission_state = MissionState.IDLE
        self.vehicle_status = None
        self.battery_status = None
        self.local_position = None
        self.last_heartbeat = None
        self.last_telemetry = None
        self.mission_start_time = None
        self.emergency_actions_taken = []
        self.data_lock = Lock()
        self.last_command_result = None
        self.node_start_time = time.time()
        
        # NEW: Command rate limiting
        self.last_command_time = 0
        self.pending_mode_change = None
        self.mode_change_attempts = 0
        self.max_mode_change_attempts = 3
        
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
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile=px4_qos_profile
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            qos_profile=px4_qos_profile
        )
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile=px4_qos_profile
        )
        
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            qos_profile=px4_qos_profile
        )
        
        # Publishers
        self.mission_status_pub = self.create_publisher(
            String,
            '/seawings/mission_status',
            10
        )
        
        # Timer for periodic mission supervision
        self.timer = self.create_timer(self.check_interval, self.supervise_mission)
        
        self.get_logger().info('MissionSupervisor node initialized with rate limiting')
        self.get_logger().info(f'Critical battery threshold: {self.critical_battery}%')
        self.get_logger().info(f'Command cooldown: {self.command_cooldown}s')
        
    def vehicle_status_callback(self, msg):
        with self.data_lock:
            self.vehicle_status = msg
            self.last_telemetry = time.time()
            self.last_heartbeat = time.time()
            
    def battery_callback(self, msg):
        with self.data_lock:
            self.battery_status = msg
            
    def local_position_callback(self, msg):
        with self.data_lock:
            self.local_position = msg
            
    def command_ack_callback(self, msg):
        with self.data_lock:
            self.last_command_result = msg
            
            # Check if this was our pending mode change
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and self.pending_mode_change:
                if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info(f'✅ Mode change to {self.pending_mode_change} accepted')
                    self.pending_mode_change = None
                    self.mode_change_attempts = 0
                elif msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                    self.get_logger().warn(f'⏳ Mode change to {self.pending_mode_change} temporarily rejected')
                    self.mode_change_attempts += 1
                else:
                    self.get_logger().error(f'❌ Mode change to {self.pending_mode_change} failed: {msg.result}')
                    self.mode_change_attempts += 1
                    
                # Give up after max attempts
                if self.mode_change_attempts >= self.max_mode_change_attempts:
                    self.get_logger().error(f'Giving up on mode change after {self.mode_change_attempts} attempts')
                    self.pending_mode_change = None
                    self.mode_change_attempts = 0
    
    def get_battery_percentage(self):
        """Get battery percentage handling both 0-1 and 0-100 scales"""
        if self.battery_status is None:
            return None
            
        if hasattr(self.battery_status, 'remaining'):
            if self.battery_status.remaining <= 1.0:
                return self.battery_status.remaining * 100
            else:
                return self.battery_status.remaining
        return None
    
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
    
    def is_armed(self):
        if not self.vehicle_status:
            return False
        return self.vehicle_status.arming_state == 2
    
    def is_in_startup_grace_period(self):
        """Check if still in startup grace period"""
        return (time.time() - self.node_start_time) < self.startup_grace
    
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send command with rate limiting"""
        current_time = time.time()
        
        # Check cooldown
        if current_time - self.last_command_time < self.command_cooldown:
            time_left = self.command_cooldown - (current_time - self.last_command_time)
            self.get_logger().debug(f'Command cooldown active ({time_left:.1f}s remaining)')
            return False
        
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
        self.last_command_time = current_time
        return True
            
    def supervise_mission(self):
        """Main mission supervision logic"""
        with self.data_lock:
            current_time = time.time()
            
            # Update mission state
            self.update_mission_state()
            
            # Retry pending mode changes
            if self.pending_mode_change and (current_time - self.last_command_time) >= self.command_cooldown:
                self.retry_mode_change()
            
            # Skip emergency checks during startup grace period
            if not self.is_in_startup_grace_period():
                # Check for communication timeouts
                self.check_communication_health(current_time)
                
                # Check mission duration
                self.check_mission_duration(current_time)
                
                # Check for emergency conditions
                self.check_emergency_conditions()
            
            # Always publish status and log
            self.publish_mission_status()
            self.log_mission_state()
            
    def update_mission_state(self):
        if self.vehicle_status is None:
            return
            
        old_state = self.mission_state
        current_mode = self.get_current_mode()
        is_armed = self.is_armed()
        
        # State machine logic
        if not is_armed:
            self.mission_state = MissionState.DISARMED
        elif current_mode == 'AUTO.TAKEOFF':
            self.mission_state = MissionState.TAKEOFF
        elif current_mode in ['AUTO.MISSION', 'AUTO.LOITER']:
            self.mission_state = MissionState.MISSION
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
        elif current_mode == 'OFFBOARD':
            self.mission_state = MissionState.MISSION
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
        elif current_mode == 'AUTO.RTL':
            self.mission_state = MissionState.RTL
        elif current_mode in ['AUTO.LAND', 'AUTO.PRECLAND', 'AUTO.LAND_ENGAGED']:
            self.mission_state = MissionState.LANDING
        elif current_mode in ['STABILIZED', 'ALTITUDE', 'POSITION'] and is_armed:
            self.mission_state = MissionState.ARMED
        elif is_armed:
            self.mission_state = MissionState.ARMED
        else:
            self.mission_state = MissionState.IDLE
            
        # Log state transitions
        if old_state != self.mission_state:
            self.get_logger().info(f'Mission state changed: {old_state.name} -> {self.mission_state.name}')
            
    def check_communication_health(self, current_time):
        if self.is_failsafe_active():
            self.get_logger().info('PX4 failsafe active - letting it handle the situation')
            return
        
        # Check heartbeat
        if self.last_heartbeat is not None:
            heartbeat_age = current_time - self.last_heartbeat
            if heartbeat_age > self.heartbeat_timeout:
                self.handle_communication_loss("Vehicle status timeout")
                
        # Check telemetry
        if self.last_telemetry is not None:
            telemetry_age = current_time - self.last_telemetry
            if telemetry_age > self.telemetry_timeout:
                self.handle_communication_loss("Telemetry timeout")
                
    def check_mission_duration(self, current_time):
        if self.mission_start_time is not None:
            mission_duration = current_time - self.mission_start_time
            if mission_duration > self.mission_timeout:
                self.handle_mission_timeout()
                
    def check_emergency_conditions(self):
        if self.battery_status is None:
            return
            
        battery_percentage = self.get_battery_percentage()
        if battery_percentage is None:
            return
            
        # Check for truly critical battery level
        if battery_percentage < self.critical_battery:
            self.handle_critical_battery()
            
        # Check for failsafe
        if self.vehicle_status is not None and self.vehicle_status.failsafe:
            self.handle_failsafe_activation()
                
    def handle_communication_loss(self, reason):
        if "comm_loss" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("comm_loss")
            self.get_logger().error(f'Communication loss detected: {reason}')
            
            if self.mission_state not in [MissionState.RTL, MissionState.LANDING, MissionState.EMERGENCY]:
                self.trigger_mode_change('RTL', 'Communication loss')
                
    def handle_mission_timeout(self):
        if "mission_timeout" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("mission_timeout")
            self.get_logger().warn('Mission timeout - triggering RTL')
            self.trigger_mode_change('RTL', 'Mission timeout')
            
    def handle_critical_battery(self):
        if "critical_battery" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("critical_battery")
            battery_pct = self.get_battery_percentage()
            self.get_logger().error(f'CRITICAL BATTERY ({battery_pct:.1f}%) - triggering emergency landing')
            self.trigger_mode_change('LAND', 'Critical battery')
            
    def handle_failsafe_activation(self):
        if "failsafe" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("failsafe")
            self.get_logger().error('PX4 failsafe activated')
            
    def trigger_mode_change(self, mode, reason):
        """Trigger mode change with rate limiting"""
        # Check if already in target mode
        current_mode = self.get_current_mode()
        if mode == 'RTL' and current_mode == 'AUTO.RTL':
            self.get_logger().debug('Already in RTL mode')
            return
        elif mode == 'LAND' and current_mode in ['AUTO.LAND', 'AUTO.LAND_ENGAGED']:
            self.get_logger().debug('Already in LAND mode')
            return
        
        # Don't override pending changes
        if self.pending_mode_change:
            self.get_logger().debug(f'Mode change to {mode} requested but {self.pending_mode_change} is pending')
            return
        
        self.get_logger().info(f'Triggering mode change to {mode}: {reason}')
        
        mode_map = {
            'RTL': 5.0,
            'LAND': 6.0,
            'MISSION': 4.0,
            'LOITER': 3.0,
            'POSITION': 2.0
        }
        
        if mode in mode_map:
            if self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=mode_map[mode]
            ):
                self.pending_mode_change = mode
                self.mode_change_attempts = 1
        else:
            self.get_logger().error(f'Unknown mode: {mode}')
    
    def retry_mode_change(self):
        """Retry pending mode change"""
        if not self.pending_mode_change:
            return
            
        self.get_logger().info(f'Retrying mode change to {self.pending_mode_change} (attempt {self.mode_change_attempts + 1})')
        
        mode_map = {
            'RTL': 5.0,
            'LAND': 6.0,
            'MISSION': 4.0,
            'LOITER': 3.0,
            'POSITION': 2.0
        }
        
        if self.pending_mode_change in mode_map:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=mode_map[self.pending_mode_change]
            )
            
    def arm_vehicle(self):
        if self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        ):
            self.get_logger().info('Arm command sent')
        
    def disarm_vehicle(self):
        if self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        ):
            self.get_logger().info('Disarm command sent')
            
    def publish_mission_status(self):
        msg = String()
        msg.data = f'{self.mission_state.name}'
        self.mission_status_pub.publish(msg)
        
    def log_mission_state(self):
        if self.mission_state == MissionState.MISSION and self.mission_start_time is not None:
            mission_duration = time.time() - self.mission_start_time
            battery_pct = self.get_battery_percentage()
            if battery_pct is None:
                battery_pct = 0
            current_mode = self.get_current_mode()
            
            self.get_logger().info(
                f'Mission Status - State: {self.mission_state.name}, '
                f'Duration: {mission_duration:.1f}s, '
                f'Battery: {battery_pct:.1f}%, '
                f'Mode: {current_mode}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
