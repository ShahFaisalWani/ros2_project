#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info("Encoder Node Initialized")
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_transforms)  # Timer set for 10 Hz

        # Set up GPIO for encoder inputs
        self.setup_gpio_encoders()
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_time = self.get_clock().now()

        # Constants for your robot
        self.wheel_radius = 0.1  # in meters
        self.robot_track = 0.5   # distance between wheels in meters

    def setup_gpio_encoders(self):
        # Encoder GPIO pins
        self.left_encoder_A = 5  # Example pin for channel A
        self.left_encoder_B =  6 # Example pin for channel B
        self.right_encoder_A = 26  # Example pin for channel A
        self.right_encoder_B = 16  # Example pin for channel B

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_encoder_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.left_encoder_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_encoder_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_encoder_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Attach interrupt handlers
        GPIO.add_event_detect(self.left_encoder_A, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.right_encoder_A, GPIO.BOTH, callback=self.right_encoder_callback)

    def left_encoder_callback(self, channel):
        if GPIO.input(self.left_encoder_A) == GPIO.input(self.left_encoder_B):
            self.left_encoder_count -= 1
        else:
            self.left_encoder_count += 1

    def right_encoder_callback(self, channel):
        if GPIO.input(self.right_encoder_A) == GPIO.input(self.right_encoder_B):
            self.right_encoder_count -= 1
        else:
            self.right_encoder_count += 1

    def publish_transforms(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate distance moved
        left_distance = 2 * math.pi * self.wheel_radius * self.left_encoder_count
        right_distance = 2 * math.pi * self.wheel_radius * self.right_encoder_count
        distance = (left_distance + right_distance) / 2

        # Calculate change in orientation
        theta = (right_distance - left_distance) / self.robot_track

        # Calculate new position in the odom frame
        x = distance * math.cos(theta)
        y = distance * math.sin(theta)

        # Reset encoder counts
        self.left_encoder_count = 0
        self.right_encoder_count = 0

        # Create the transformation
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(theta / 2)
        t.transform.rotation.w = math.cos(theta / 2)

        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
