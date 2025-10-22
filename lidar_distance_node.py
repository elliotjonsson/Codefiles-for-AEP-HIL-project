#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

class LidarDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_distance_node')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.lidar_callback,
            qos
        )
        self.distance_pub = self.create_publisher(Float32, '/lidar/min_distance', 1)
        self.get_logger().info('Lidar distance node started.')

    def lidar_callback(self, msg):
        # Filtrera bort inf och nan
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < 30.0]
        if valid_ranges:
            min_dist = min(valid_ranges)
            min_dist_cm = min_dist * 100
            # Publicera minsta avståndet i cm
            msg_out = Float32()
            msg_out.data = float(min_dist_cm)
            self.distance_pub.publish(msg_out)
            if min_dist_cm < 20:
                self.get_logger().warn('WARNING, watch out for obstacles')
            else:
                self.get_logger().info(f'Distance: {min_dist_cm:.0f} cm')
        else:
            self.get_logger().info('No measurment.')

def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
