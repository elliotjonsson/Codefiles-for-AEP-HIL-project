#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32


class LidarDistanceNode(Node):
    """
    Read /scan_raw (LaserScan) and publish the minimum valid distance
    in a 90-degree FRONT sector as centimeters on /lidar/min_distance.

    The front sector is defined as +/- 45 degrees around 0 radians
    (assuming 0 rad is "straight ahead" of the robot).
    """

    def __init__(self):
        super().__init__("lidar_distance_node")

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.lidar_callback,
            qos,
        )

        self.min_dist_pub = self.create_publisher(
            Float32,
            "/lidar/min_distance",
            1,
        )

        self.get_logger().info("LidarDistanceNode started (front 90° sector only).")

    def lidar_callback(self, msg: LaserScan):
        # Half-angle of the front sector: 45 degrees in radians
        front_half_angle = math.radians(45.0)

        valid_ranges = []

        # Walk through all ranges and select only those in the front 90° sector
        angle = msg.angle_min
        for r in msg.ranges:
            # Filter out invalid or out-of-range values
            if not (0.05 < r < msg.range_max):
                angle += msg.angle_increment
                continue

            # Normalize angle to [-pi, pi]
            a = math.atan2(math.sin(angle), math.cos(angle))

            # Check if this reading is within +/-45° around 0 rad (front)
            if abs(a) <= front_half_angle:
                valid_ranges.append(r)

            angle += msg.angle_increment

        # If we found at least one valid reading in the front sector, use its minimum
        if valid_ranges:
            min_dist_m = min(valid_ranges)
            min_dist_cm = float(min_dist_m * 100.0)
        else:
            # No valid reading in the front sector: treat as "very far"
            min_dist_cm = float(msg.range_max * 100.0)

        msg_out = Float32()
        msg_out.data = min_dist_cm
        self.min_dist_pub.publish(msg_out)

        # Debug logging (throttled to avoid spamming)
        self.get_logger().info(f"Front min distance: {min_dist_cm:.0f} cm", throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
