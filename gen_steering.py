#!/usr/bin/env python3

import socket
import json
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, Bool
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState


HOST = "0.0.0.0"
PORT = 6005  # must match the client


class GenSteeringNode(Node):
    def __init__(self):
        super().__init__("gen_steering")

        # ======= Parameters =======
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # Robot geometry
        self.wheelbase = 0.145  # m

        # Emergency brake parameters (forward)
        self.t_reaction = 0.25      # reaction time in seconds
        self.a_max = 1.0            # max deceleration in m/s^2
        self.robot_offset = 0.15    # Lidar offset distance to front (meters)
        self.safety_margin = 0.0    # safety margin (meters)

        # Initialize with default minimum stop distance
        self.declare_parameter("min_stop_distance_cm", 15.0)
        self.min_stop_distance_cm = float(self.get_parameter("min_stop_distance_cm").value)
        self.stop_distance_cm = self.min_stop_distance_cm

        # State
        self.obstacle_close = False
        self.last_distance_cm = None
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.current_gear = 0
        self.min_value = 0.05  # deadzone
        
        # Latency handling
        self.last_command_time = 0.0
        self.command_timeout_sec = 120.0  # Safety stop if no command for 120 seconds
        self.last_valid_throttle = 0.0
        self.last_valid_brake = 0.0
        self.last_valid_steering = 0.0

        # Publishers
        self.drive_pub = self.create_publisher(
            Twist,
            "/controller/cmd_vel",
            10
        )

        self.servo_pub = self.create_publisher(
            SetPWMServoState,
            "ros_robot_controller/pwm_servo/set_state",
            1,
        )
        
        self.gear_pub = self.create_publisher(
            Int32,
            "/steering/current_gear",
            10
        )
        
        self.emergency_brake_pub = self.create_publisher(
            Bool,
            "/steering/emergency_brake_active",
            10
        )

        # LIDAR subscription (front 90° minimum in cm)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(
            Float32,
            "/lidar/min_distance",
            self.lidar_callback,
            qos
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST, PORT))
        self.sock.settimeout(0.05)

        self.get_logger().info("GenSteeringNode started.")
        self.get_logger().info(f"Listening on UDP {HOST}:{PORT}")
        self.get_logger().info(f"Max linear: {self.max_linear} m/s, Max angular: {self.max_angular} rad/s")
        self.get_logger().info(f"AEB enabled: min_stop={self.min_stop_distance_cm}cm, reaction={self.t_reaction}s, decel={self.a_max}m/s²")

    # ======= AEB BRAKE DISTANCE CALCULATION =======

    def calculate_stop_distance(self):
        """
        Calculate required braking distance based on current speed,
        reaction time, deceleration capacity, and safety margins.
        
        d_stop = v * t_reaction + v² / (2 * a_max)
        d_trigger = d_stop + robot_offset + safety_margin
        
        Returns distance in cm.
        """
        # Convert normalized speed to actual speed (m/s)
        v = abs(self.current_speed) * self.max_linear
        
        # Calculate stopping distance (meters)
        d_stop = (v * self.t_reaction) + ((v ** 2) / (2 * self.a_max))
        d_trigger = d_stop + self.robot_offset + self.safety_margin
        
        # Convert to cm and apply minimum
        d_trigger_cm = d_trigger * 100.0
        return max(d_trigger_cm, self.min_stop_distance_cm)

    # ======= LIDAR CALLBACK =======

    def lidar_callback(self, msg: Float32):
        """
        Receive front minimum distance (cm) from LidarDistanceNode
        and update emergency brake state.
        """
        self.last_distance_cm = msg.data
        
        # Update stop distance based on current speed (AEB calculation)
        self.stop_distance_cm = self.calculate_stop_distance()
        
        prev = self.obstacle_close
        self.obstacle_close = self.last_distance_cm < self.stop_distance_cm

        if self.obstacle_close and not prev:
            self.get_logger().warn(
                f"EMERGENCY BRAKE ON (front distance {self.last_distance_cm:.1f} cm, "
                f"required {self.stop_distance_cm:.1f} cm at speed {self.current_speed:.2f})"
            )
        elif not self.obstacle_close and prev:
            self.get_logger().info(
                f"Emergency brake released (front distance {self.last_distance_cm:.1f} cm)"
            )
        
        # Publish emergency brake status
        brake_msg = Bool()
        brake_msg.data = self.obstacle_close
        self.emergency_brake_pub.publish(brake_msg)

    # ======= DRIVE & STEERING (Ackermann like steering_control.py) =======

    def send_drive_and_steering(self, throttle: float, brake: float, steering: float):
        """
        throttle, brake, steering in [-1, 1] or [0, 1]
        Uses Ackermann steering model (MentorPi_Acker)
        """
        # Apply brake reduction to throttle (like steering_control.py)
        # Brake reduces speed in all directions
        throttle = throttle * (1.0 - brake)
        
        # Save current speed for AEB calculation
        self.current_speed = throttle
        self.current_steering = steering

        # Apply deadzone
        if abs(steering) < self.min_value:
            steering = 0.0
        if abs(throttle) < self.min_value:
            throttle = 0.0
        
        # Emergency brake: block forward movement if obstacle is close
        if self.obstacle_close and throttle > 0.0:
            throttle = 0.0
        
        twist = Twist()
        
        self.get_logger().info(f"Throttle info: {throttle}")
        # Map throttle to linear velocity
        twist.linear.x = throttle * self.max_linear
        
        # Convert steering input to steering angle (inverted)
        steering_angle = steering * math.radians(322.0 / 2000.0 * 180.0)
        self.get_logger().info(f"Steering info: {steering_angle}")
        
        if steering_angle == 0:
            twist.angular.z = 0.0
            # Center steering servo
            servo_state = PWMServoState()
            servo_state.id = [3]
            servo_state.position = [1500]
            data = SetPWMServoState()
            data.state = [servo_state]
            data.duration = 0.02
            self.servo_pub.publish(data)
        else:
            # Calculate angular velocity based on wheelbase
            R = self.wheelbase / math.tan(steering_angle)
            twist.angular.z = float(twist.linear.x / R) if R != 0 else 0.0
            
            # Set servo position based on steering angle
            servo_state = PWMServoState()
            servo_state.id = [3]
            servo_state.position = [1500 + int(math.degrees(-steering_angle) / 180.0 * 2000)]
            data = SetPWMServoState()
            data.state = [servo_state]
            data.duration = 0.02
            self.servo_pub.publish(data)
        
        # Publish motor commands
        self.drive_pub.publish(twist)

    # ======= UDP handling =======

    def parse_udp_message(self, data: bytes) -> Tuple[float, float, float, int]:
        """
        Parse JSON message from the client.
        {"throttle": ..., "brake": ..., "steering": ..., "gear": ...}
        """
        msg = json.loads(data.decode("utf-8"))
        throttle = float(msg.get("throttle", 0.0))
        brake = float(msg.get("brake", 0.0))
        steering = float(msg.get("steering", 0.0))
        gear = int(msg.get("gear", 0))
        return throttle, brake, steering, gear

    def run(self):
        self.get_logger().info("Main loop started.")
        import time
        self.last_command_time = time.time()
        
        try:
            while rclpy.ok():
                # Handle ROS callbacks (LIDAR, etc.)
                rclpy.spin_once(self, timeout_sec=0.0)

                try:
                    data, addr = self.sock.recvfrom(1024)
                    
                    # Parse new command
                    try:
                        throttle, brake, steering, gear = self.parse_udp_message(data)
                    except Exception as e:
                        self.get_logger().warn(f"Error parsing UDP data: {e}")
                        continue

                    # Update last command time and store valid command
                    self.last_command_time = time.time()
                    self.last_valid_throttle = throttle
                    self.last_valid_brake = brake
                    self.last_valid_steering = steering

                    if gear != self.current_gear:
                        self.get_logger().info(f"Gear changed to {gear}")
                        self.current_gear = gear
                        # Publish gear change
                        gear_msg = Int32()
                        gear_msg.data = gear
                        self.gear_pub.publish(gear_msg)

                    self.send_drive_and_steering(throttle, brake, steering)
                    
                except socket.timeout:
                    # No new data - check if we should continue or stop
                    time_since_last = time.time() - self.last_command_time
                    
                    if time_since_last > self.command_timeout_sec:
                        # No command for too long -> safety stop
                        self.get_logger().warn(
                            f"No command for {time_since_last:.2f}s - safety stop",
                            throttle_duration_sec=1.0
                        )
                        self.send_drive_and_steering(0.0, 0.0, 0.0)
                        self.last_valid_throttle = 0.0
                        self.last_valid_brake = 0.0
                        self.last_valid_steering = 0.0
                    else:
                        # Keep sending last valid command during network gaps
                        self.send_drive_and_steering(
                            self.last_valid_throttle,
                            self.last_valid_brake,
                            self.last_valid_steering
                        )
                    continue
                    
                except Exception as e:
                    self.get_logger().error(f"UDP error: {e}")
                    continue

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down (Ctrl+C).")
            self.send_drive_and_steering(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    node = GenSteeringNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
