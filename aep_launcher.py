#!/usr/bin/env python3
"""
AEP Package VR Launcher
Launches all nodes in the correct order with proper initialization sequence.

Node Launch Order:
1. lidar_distance - Processes LIDAR data for obstacle detection
2. gen_steering - Generates steering commands with emergency brake logic
3. message_node - Collects and formats overlay messages (embedded in stream node)
4. vr_listener - Listens to VR headset input for camera servo control
5. stream - Camera stream with HTTP server and overlays (includes message_node)
"""

import time
import subprocess
import sys
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class AEPLauncher(Node):
    def __init__(self):
        super().__init__('aep_launcher')
        self.processes = []
        self.get_logger().info("=== AEP Package VR Launcher Starting ===")
        
    def launch_node(self, node_name, delay=0.5):
        """Launch a single node as a subprocess"""
        self.get_logger().info(f"Launching {node_name}...")
        try:
            process = subprocess.Popen(
                ['ros2', 'run', 'aep_package_vr', node_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(process)
            time.sleep(delay)
            self.get_logger().info(f"✓ {node_name} started (PID: {process.pid})")
        except Exception as e:
            self.get_logger().error(f"✗ Failed to launch {node_name}: {e}")
            
    def launch_all_nodes(self):
        """Launch all nodes in the correct sequence"""
        # 1. Start LIDAR distance processing first (provides obstacle detection)
        self.launch_node('lidar_distance', delay=1.0)
        
        # 2. Start steering generator (uses LIDAR data for emergency brake)
        self.launch_node('gen_steering', delay=0.5)
        
        # 3. Start message node (collects data for overlays)
        self.launch_node('message_node', delay=0.5)
        
        # 4. Start VR listener (camera servo control)
        self.launch_node('vr_listener', delay=0.5)
        
        # 5. Start camera stream (uses all previous data)
        self.launch_node('stream', delay=1.0)
        
        self.get_logger().info("=== All nodes launched successfully ===")
        self.get_logger().info("Camera stream: http://localhost:5000/video_feed (MJPEG)")
        self.get_logger().info("Camera snapshot: http://localhost:5000/snapshot (VR polling)")
        self.get_logger().info("VR control listening on UDP port 5005")
        self.get_logger().info("Steering control listening on TCP port 6005")
        self.get_logger().info("Press Ctrl+C to stop all nodes")
        
    def shutdown_all(self):
        """Terminate all child processes"""
        self.get_logger().info("Shutting down all nodes...")
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=3)
            except Exception as e:
                self.get_logger().warn(f"Force killing process {process.pid}")
                process.kill()
        self.get_logger().info("All nodes stopped")


def main(args=None):
    rclpy.init(args=args)
    launcher = AEPLauncher()
    
    # Launch all nodes
    launcher.launch_all_nodes()
    
    try:
        # Keep launcher alive and monitor
        rclpy.spin(launcher)
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        launcher.shutdown_all()
        launcher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
