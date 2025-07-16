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
        self.declare_parameter('safety_margin', 0.3)  # 30% - increased for safety
        self.declare_parameter('average_return_speed', 8.0)  # m/s - conservative estimate
        self.declare_parameter('battery_check_interval', 3.0)  # seconds - more frequent
        self.declare_parameter('min_battery_voltage', 14.0)  # volts
        self.declare_parameter('rtl_triggered_threshold', 0.25)  # 25% battery
        self.declare_parameter('battery_capacity_mah', 5000.0)  # Fallback capacity
        self.declare_parameter('min_current_threshold', 0.1)  # Minimum current for calculations (A)
        self.declare_parameter('current_averaging_window', 10)  # Number of samples for averaging
        self.declare_parameter('home_position_timeout', 30.0)  # Max time to wait for home position

        self.safety_margin = self.get_parameter('safety_margin').value
        self.return_speed = self.get_parameter('average_return_speed').value
        self.check_interval = self.get_parameter('battery_check_interval').value
        self.rtl_threshold = self.get_parameter('rtl_triggered_threshold').value
        self.battery_capacity = self.get_parameter('battery_capacity_mah').value
        self.min_current = self.get_parameter('min_current_threshold').value
        self.current_window_size = self.get_parameter('current_averaging_window').value
        self.home_timeout = self.get_parameter('home_position_timeout').value

        # State
        self.battery_status = None
        self.global_position = None
        self.home_position = None  # (lat, lon, alt)
        self.vehicle_status = None
        self.rtl_triggered = False
        self.lock = Lock()
        self.start_time = time.time()
        self.last_command_result = None
        
        # Current averaging for more stable calculations
        self.current_history = deque(maxlen=self.current_window_size)

        # Define QoS profile for PX4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers for direct PX4 control (no MAVROS needed!)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos_profile
        )

        # Subscribers with PX4 QoS compatibility
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.gps_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.command_ack_callback, qos_profile=px4_qos_profile)

        # Periodic check
        self.create_timer(self.check_interval, self.check_battery_status)

        self.get_logger().info('✅ PowerMonitor node initialized with direct PX4 control (no MAVROS needed!)')

    def battery_callback(self, msg):
        with self.lock:
            self.battery_status = msg
            # Store current history for averaging
            if msg.current_a > 0:
                self.current_history.append(msg.current_a)

    def gps_callback(self, msg):
        with self.lock:
            self.global_position = msg
            # Better home position detection - only set if we have good GPS
            if (self.home_position is None and 
                not msg.dead_reckoning and 
                abs(msg.lat) > 0.001 and abs(msg.lon) > 0.001 and  # Not zero coordinates
                abs(msg.lat) <= 90.0 and abs(msg.lon) <= 180.0):  # Valid coordinate range
                
                self.home_position = (msg.lat, msg.lon, msg.alt)
                self.get_logger().info(f'🏠 Home position set: lat={msg.lat:.6f}, lon={msg.lon:.6f}, alt={msg.alt:.2f} m')

    def status_callback(self, msg):
        with self.lock:
            self.vehicle_status = msg

    def command_ack_callback(self, msg):
        """Handle command acknowledgments from PX4"""
        with self.lock:
            self.last_command_result = msg
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.get_logger().info('✅ RTL mode change accepted by PX4')
                else:
                    self.get_logger().error(f'❌ RTL mode change failed: result={msg.result}')
                    self.rtl_triggered = False  # Allow retry

    def get_current_mode(self):
        """Get current flight mode from vehicle status"""
        if not self.vehicle_status:
            return "UNKNOWN"
        
        # PX4 nav_state to mode name mapping
        mode_map = {
            0: "MANUAL",
            1: "ALTITUDE",
            2: "POSITION", 
            3: "AUTO.LOITER",
            4: "AUTO.MISSION",
            5: "AUTO.RTL",
            6: "AUTO.LAND",
            7: "AUTO.TAKEOFF",
            8: "OFFBOARD",
            9: "STABILIZED",
            10: "ACRO"
        }
        
        return mode_map.get(self.vehicle_status.nav_state, f"UNKNOWN({self.vehicle_status.nav_state})")

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send command directly to PX4 via uORB"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
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

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius (m)
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

        # Calculate horizontal distance
        horizontal = self.haversine_distance(lat1, lon1, lat2, lon2)
        
        # For return time, use horizontal distance + altitude consideration
        # (drone will likely return at current altitude then descend)
        vertical = abs(alt2 - alt1)
        
        # Return conservative estimate - horizontal distance + some vertical component
        return horizontal + (vertical * 0.5)  # Assume 50% of vertical distance adds to travel

    def get_average_current(self):
        """Get averaged current draw for more stable calculations"""
        if not self.current_history:
            return None
        return sum(self.current_history) / len(self.current_history)

    def calculate_remaining_time(self):
        if self.battery_status is None:
            return None

        # Use averaged current for stability
        avg_current = self.get_average_current()
        if avg_current is None or avg_current < self.min_current:
            return None

        # Use actual battery capacity if available, otherwise use parameter
        if hasattr(self.battery_status, 'capacity') and self.battery_status.capacity > 0:
            capacity = self.battery_status.capacity
        else:
            capacity = self.battery_capacity

        percent = self.battery_status.remaining
        remaining_mah = (percent / 100.0) * capacity
        current_draw_ma = avg_current * 1000.0

        time_hours = remaining_mah / current_draw_ma
        return time_hours * 3600.0  # seconds

    def check_battery_status(self):
        with self.lock:
            if self.rtl_triggered:
                return

            # Check if we're still waiting for home position
            if self.home_position is None:
                elapsed = time.time() - self.start_time
                if elapsed > self.home_timeout:
                    self.get_logger().warn(f'⚠️ No home position after {self.home_timeout}s - using current position')
                    if self.global_position is not None:
                        self.home_position = (self.global_position.lat, self.global_position.lon, self.global_position.alt)
                return

            # Skip if not in appropriate flight mode
            current_mode = self.get_current_mode()
            if current_mode in ['AUTO.RTL', 'AUTO.LAND', 'MANUAL']:
                return

            if not all([self.battery_status, self.global_position]):
                return

            remaining_time = self.calculate_remaining_time()
            distance = self.calculate_distance_to_home()

            if remaining_time is None or distance is None:
                self.get_logger().warn('⚠️ Cannot calculate battery remaining time or distance')
                return

            # Calculate return time with safety margin
            base_return_time = distance / self.return_speed
            safety_return_time = base_return_time * (1 + self.safety_margin)

            # Add extra buffer for altitude changes and maneuvering
            safety_return_time += 60.0  # Extra 60 seconds buffer

            self.get_logger().info(
                f'🔋 Battery: {self.battery_status.remaining:.1f}%, '
                f'⏳ Remaining: {remaining_time:.1f}s, '
                f'🏠 Distance: {distance:.1f}m, '
                f'🔁 Needed: {safety_return_time:.1f}s, '
                f'⚡ Current: {self.get_average_current():.2f}A, '
                f'📍 Mode: {current_mode}'
            )

            # Trigger RTL if either condition is met
            if (remaining_time < safety_return_time or 
                self.battery_status.remaining < self.rtl_threshold * 100):
                
                reason = "insufficient time" if remaining_time < safety_return_time else "low battery threshold"
                self.trigger_rtl(reason)

    def trigger_rtl(self, reason="battery low"):
        if self.rtl_triggered:
            return
        
        self.rtl_triggered = True
        self.get_logger().warn(f'⚠️ LOW BATTERY - Triggering Return-to-Launch: {reason}')
        
        # Send RTL mode command directly to PX4
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # base mode (1 = custom mode)
            param2=5.0   # custom mode (5 = RTL for PX4)
        )
        
        self.get_logger().info('📡 RTL command sent directly to PX4 (no MAVROS needed!)')

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