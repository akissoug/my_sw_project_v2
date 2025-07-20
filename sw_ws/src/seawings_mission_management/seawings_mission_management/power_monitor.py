#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import BatteryStatus, VehicleGlobalPosition, VehicleStatus, VehicleCommand, VehicleCommandAck
import math
from threading import Lock
from collections import deque
import time

class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')

        # Parameters
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('average_return_speed', 8.0)
        self.declare_parameter('battery_check_interval', 3.0)
        self.declare_parameter('min_battery_voltage', 14.0)
        self.declare_parameter('rtl_triggered_threshold', 0.25)
        self.declare_parameter('battery_capacity_mah', 5000.0)
        self.declare_parameter('min_current_threshold', 0.1)
        self.declare_parameter('current_averaging_window', 10)
        self.declare_parameter('home_position_timeout', 30.0)
        self.declare_parameter('sitl_mode', True)
        self.declare_parameter('sitl_battery_drain_rate', 0.2)  # % per minute in SITL

        self.safety_margin = self.get_parameter('safety_margin').value
        self.return_speed = self.get_parameter('average_return_speed').value
        self.check_interval = self.get_parameter('battery_check_interval').value
        self.rtl_threshold = self.get_parameter('rtl_triggered_threshold').value
        self.battery_capacity = self.get_parameter('battery_capacity_mah').value
        self.min_current = self.get_parameter('min_current_threshold').value
        self.current_window_size = self.get_parameter('current_averaging_window').value
        self.home_timeout = self.get_parameter('home_position_timeout').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        self.sitl_drain_rate = self.get_parameter('sitl_battery_drain_rate').value

        # State
        self.battery_status = None
        self.global_position = None
        self.home_position = None
        self.vehicle_status = None
        self.rtl_triggered = False
        self.lock = Lock()
        self.start_time = time.time()
        self.last_command_result = None
        self.command_retry_count = 0
        self.max_command_retries = 3
        self.last_command_time = 0
        self.command_cooldown = 5.0  # seconds between retry attempts
        
        # SITL battery simulation
        self.sitl_battery_start = 100.0
        self.armed_time = None
        
        # Current averaging
        self.current_history = deque(maxlen=self.current_window_size)
        self.sitl_default_current = 10.0

        # Define QoS profile
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos_profile
        )

        # Subscribers
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.gps_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.command_ack_callback, qos_profile=px4_qos_profile)

        # Periodic check
        self.create_timer(self.check_interval, self.check_battery_status)

        self.get_logger().info('✅ PowerMonitor node initialized')
        if self.sitl_mode:
            self.get_logger().info(f'🎮 SITL mode: simulating battery drain at {self.sitl_drain_rate}% per minute')

    def battery_callback(self, msg):
        with self.lock:
            self.battery_status = msg
            current = msg.current_a if msg.current_a > 0 else self.sitl_default_current
            self.current_history.append(current)

    def gps_callback(self, msg):
        with self.lock:
            self.global_position = msg
            if (self.home_position is None and 
                not msg.dead_reckoning and 
                abs(msg.lat) > 0.001 and abs(msg.lon) > 0.001 and
                abs(msg.lat) <= 90.0 and abs(msg.lon) <= 180.0):
                
                self.home_position = (msg.lat, msg.lon, msg.alt)
                self.get_logger().info(f'🏠 Home position set: lat={msg.lat:.6f}, lon={msg.lon:.6f}, alt={msg.alt:.2f} m')

    def status_callback(self, msg):
        with self.lock:
            self.vehicle_status = msg
            
            # Track armed time for SITL battery simulation
            if self.sitl_mode and msg.arming_state == 2:  # Armed
                if self.armed_time is None:
                    self.armed_time = time.time()
                    self.get_logger().info('🚁 Vehicle armed - starting battery simulation')
            elif self.armed_time is not None:
                self.armed_time = None
                self.get_logger().info('🚁 Vehicle disarmed - stopping battery simulation')

    def command_ack_callback(self, msg):
        with self.lock:
            self.last_command_result = msg
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info('✅ RTL mode change accepted by PX4')
                    self.command_retry_count = 0
                    self.rtl_triggered = True
                elif msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                    self.get_logger().warn('⏳ RTL command temporarily rejected - will retry')
                    # Don't increment retry count for temporary rejection
                else:
                    self.get_logger().error(f'❌ RTL mode change failed: result={msg.result}')
                    self.command_retry_count += 1
                    if self.command_retry_count >= self.max_command_retries:
                        self.get_logger().error('❌ Max retries reached - giving up on RTL command')
                        self.rtl_triggered = True  # Don't keep trying

    def get_current_mode(self):
        if not self.vehicle_status:
            return "UNKNOWN"
        
        mode_map = {
            0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "AUTO.LOITER",
            4: "AUTO.MISSION", 5: "AUTO.RTL", 6: "AUTO.LAND", 7: "AUTO.TAKEOFF",
            8: "OFFBOARD", 9: "STABILIZED", 10: "ACRO", 11: "AUTO.LAND_ENGAGED",
            12: "AUTO.PRECLAND", 13: "ORBIT", 14: "AUTO.VTOL_TAKEOFF"
        }
        
        return mode_map.get(self.vehicle_status.nav_state, f"UNKNOWN({self.vehicle_status.nav_state})")

    def is_failsafe_active(self):
        """Check if PX4 failsafe is already active"""
        if self.vehicle_status:
            return self.vehicle_status.failsafe
        return False

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        current_time = time.time()
        
        # Check cooldown
        if current_time - self.last_command_time < self.command_cooldown:
            self.get_logger().debug('Command cooldown active - skipping')
            return
            
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

    def get_simulated_battery_percentage(self):
        """Simulate battery drain in SITL mode"""
        if not self.sitl_mode or self.armed_time is None:
            return None
            
        # Calculate time armed in minutes
        armed_minutes = (time.time() - self.armed_time) / 60.0
        
        # Calculate simulated battery
        simulated_battery = self.sitl_battery_start - (armed_minutes * self.sitl_drain_rate)
        simulated_battery = max(0.0, simulated_battery)  # Don't go below 0
        
        return simulated_battery

    def get_battery_percentage(self):
        """Get battery percentage with SITL simulation override"""
        if self.battery_status is None:
            return None
            
        # In SITL mode, use simulated battery if armed
        if self.sitl_mode:
            simulated = self.get_simulated_battery_percentage()
            if simulated is not None:
                return simulated
        
        # Otherwise use real battery data
        if hasattr(self.battery_status, 'remaining'):
            if self.battery_status.remaining <= 1.0:
                return self.battery_status.remaining * 100
            else:
                return self.battery_status.remaining
        return None

    def calculate_remaining_time(self):
        battery_percentage = self.get_battery_percentage()
        if battery_percentage is None:
            return None

        avg_current = self.get_average_current()
        if avg_current is None or avg_current < self.min_current:
            # In SITL, estimate based on drain rate
            if self.sitl_mode and battery_percentage > 0:
                minutes_remaining = battery_percentage / self.sitl_drain_rate
                return minutes_remaining * 60.0  # Convert to seconds
            return None

        capacity = self.battery_capacity
        remaining_mah = (battery_percentage / 100.0) * capacity
        current_draw_ma = avg_current * 1000.0
        time_hours = remaining_mah / current_draw_ma
        return time_hours * 3600.0

    def get_average_current(self):
        if not self.current_history:
            return self.sitl_default_current if self.sitl_mode else None
        return sum(self.current_history) / len(self.current_history)

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)
        a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def calculate_distance_to_home(self):
        if self.home_position is None or self.global_position is None:
            return None
        
        lat1, lon1, alt1 = self.home_position
        lat2, lon2, alt2 = self.global_position.lat, self.global_position.lon, self.global_position.alt
        horizontal = self.haversine_distance(lat1, lon1, lat2, lon2)
        vertical = abs(alt2 - alt1)
        return horizontal + (vertical * 0.5)

    def check_battery_status(self):
        with self.lock:
            if self.rtl_triggered:
                return

            # Skip if PX4 failsafe is already active
            if self.is_failsafe_active():
                self.get_logger().debug('PX4 failsafe active - skipping battery check')
                return

            if self.home_position is None:
                elapsed = time.time() - self.start_time
                if elapsed > self.home_timeout:
                    self.get_logger().warn(f'⚠️ No home position after {self.home_timeout}s')
                return

            current_mode = self.get_current_mode()
            if current_mode in ['AUTO.RTL', 'AUTO.LAND', 'MANUAL', 'AUTO.LAND_ENGAGED', 'AUTO.PRECLAND']:
                return

            if not all([self.battery_status, self.global_position]):
                return

            battery_percentage = self.get_battery_percentage()
            if battery_percentage is None:
                return

            remaining_time = self.calculate_remaining_time()
            distance = self.calculate_distance_to_home()

            if distance is None:
                return

            # Calculate return time
            base_return_time = distance / self.return_speed
            safety_return_time = base_return_time * (1 + self.safety_margin) + 60.0

            # Log status
            self.get_logger().info(
                f'🔋 Battery: {battery_percentage:.1f}%, '
                f'⏳ Remaining: {remaining_time:.1f}s, ' if remaining_time else ''
                f'🏠 Distance: {distance:.1f}m, '
                f'🔁 Needed: {safety_return_time:.1f}s, '
                f'📍 Mode: {current_mode}'
            )

            # Check RTL conditions
            should_rtl = False
            reason = ""
            
            if battery_percentage < self.rtl_threshold * 100:
                should_rtl = True
                reason = f"low battery ({battery_percentage:.1f}%)"
            elif remaining_time is not None and remaining_time < safety_return_time:
                should_rtl = True
                reason = f"insufficient time"
            
            if should_rtl and self.command_retry_count < self.max_command_retries:
                self.trigger_rtl(reason)

    def trigger_rtl(self, reason="battery low"):
        if self.rtl_triggered:
            return
        
        self.get_logger().warn(f'⚠️ Triggering RTL: {reason}')
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=5.0
        )
        
        self.get_logger().info('📡 RTL command sent directly to PX4')

def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
