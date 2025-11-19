#!/usr/bin/env python3
# encoding: utf-8

import socket
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HOST = "0.0.0.0"   # lyssna på alla interfaces
PORT = 5005        # måste matcha g29_client.py

class WheelRemoteControl(Node):
    def __init__(self):
        super().__init__("wheel_remote_control")

        # Samma parametrar som joystick-koden använder
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.machine = os.environ.get('MACHINE_TYPE', 'unknown')
        self.get_logger().info(f"MACHINE_TYPE = {self.machine}")
        self.get_logger().info(
            f"max_linear={self.max_linear}, max_angular={self.max_angular}"
        )

        # Viktigt: samma topic som joystick-controller använder
        self.cmd_pub = self.create_publisher(Twist, 'controller/cmd_vel', 10)

    def send_cmd(self, v, omega):
        """
        v: 0..1   (från ratten, framåt-hastighet)
        omega: -1..1 (från ratten, sväng vänster/höger)
        """

        # Klipp till rimliga intervall, så vi inte skickar skräp
        v = max(0.0, min(1.0, float(v)))
        omega = max(-1.0, min(1.0, float(omega)))

        twist = Twist()
        # Enkel mapping: v → framåt, omega → rotation
        twist.linear.x = v * self.max_linear
        twist.angular.z = omega * self.max_angular

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"Publikar cmd: lin.x={twist.linear.x:.2f}, ang.z={twist.angular.z:.2f}"
        )


def handle_client(node: WheelRemoteControl, conn, addr):
    node.get_logger().info(f"Client ansluten från {addr}")
    buffer = b""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                node.get_logger().info("Client kopplade från.")
                break

            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.decode("utf-8").strip()
                if not line:
                    continue

                try:
                    v_str, omega_str = line.split(",")
                    v = float(v_str)
                    omega = float(omega_str)

                    node.get_logger().info(f"Mottog v={v:.2f}, omega={omega:.2f}")
                    node.send_cmd(v, omega)

                except ValueError:
                    node.get_logger().warn(f"Ogiltig rad: '{line}'")
    finally:
        conn.close()


def main():
    rclpy.init()
    node = WheelRemoteControl()

    # TCP-server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Fixar "Address already in use" när du startar om snabbt
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        node.get_logger().info(f"Lyssnar på {HOST}:{PORT} ...")

        try:
            while rclpy.ok():
                conn, addr = s.accept()
                handle_client(node, conn, addr)
        except KeyboardInterrupt:
            node.get_logger().info("Avslutar servern (Ctrl+C).")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
