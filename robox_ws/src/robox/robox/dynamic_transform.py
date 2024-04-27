#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class DynamicTransformPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_transform_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transforms)  # 10 Hz
        self.start_time = self.get_clock().now()

    def publish_transforms(self):
        now = self.get_clock().now()
        elapsed_time = now - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9  # Convert nanoseconds to seconds
        
        # Example transformation calculation (could be based on real sensor data)
        x = math.sin(elapsed_seconds * 0.1) * 2.0  # Simulated position
        y = math.cos(elapsed_seconds * 0.1) * 2.0
        theta = elapsed_seconds * 0.1  # Simulated orientation
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # Simple rotation around z-axis
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
