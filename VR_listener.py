#!/usr/bin/env python3
import socket
import re
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import PWMServoState, SetPWMServoState
from std_msgs.msg import Int32MultiArray  # NEW
import threading

# --- Config ---
UDP_IP = "0.0.0.0"      # allow any device to connect
UDP_PORT = 5005         # port
SERVO_ID_PITCH = 1      # vertical servo ID
SERVO_ID_YAW = 2        # horizontal servo ID
CENTER_POS = 1500       # center
RANGE = 500             # range (1000–2000)

class UDPServoNode(Node):
    def __init__(self):
        super().__init__('udp_servo_node')
        
        # ROS2 publishers
        self.pwm_pub = self.create_publisher(
            SetPWMServoState, 
            'ros_robot_controller/pwm_servo/set_state', 
            10
        )
        
        # NEW: publish raw pan/tilt PWM values for overlay node
        self.angles_pub = self.create_publisher(
            Int32MultiArray,
            '/camera_servo/angles',
            10
        )
        
        # Init UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.05)

        self.running = True
        self.get_logger().info(f"=== UDP servo receiver started (port {UDP_PORT}) ===")

        # Start receive thread
        self.thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.thread.start()

    def angle_to_pwm(self, angle, is_inverse=False):
        """Convert angle (-90..90) to PWM (1000..2000)."""
        angle = max(-90.0, min(90.0, angle))
        
        if is_inverse:
            angle = -angle
            
        pwm = CENTER_POS + (angle / 90.0) * RANGE
        return int(pwm)

    def drive_servos(self, pitch_pwm, yaw_pwm):
        # Servo command
        msg = SetPWMServoState()
        msg.duration = 0.05
        
        s1 = PWMServoState()
        s1.id = [SERVO_ID_PITCH]
        s1.position = [int(pitch_pwm)]
        
        s2 = PWMServoState()
        s2.id = [SERVO_ID_YAW]
        s2.position = [int(yaw_pwm)]
        
        msg.state = [s1, s2]
        self.pwm_pub.publish(msg)

        # NEW: publish pan/tilt PWM for overlay (pan = yaw, tilt = pitch)
        angles_msg = Int32MultiArray()
        angles_msg.data = [int(yaw_pwm), int(pitch_pwm)]
        self.angles_pub.publish(angles_msg)

    def udp_loop(self):
        while self.running and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                text = data.decode('utf-8')
                
                match = re.search(r"P:([\d\.-]+),Y:([\d\.-]+)", text)
                if match:
                    raw_pitch = float(match.group(1))
                    raw_yaw = float(match.group(2))
                    
                    pwm_pitch = self.angle_to_pwm(raw_pitch, is_inverse=False)
                    pwm_yaw = self.angle_to_pwm(raw_yaw,   is_inverse=True)
                    
                    self.drive_servos(pwm_pitch, pwm_yaw)
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = UDPServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
