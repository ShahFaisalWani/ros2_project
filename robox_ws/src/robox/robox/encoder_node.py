#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
import math
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info("Encoder Node Started")
        
        # Publisher for Odometry data
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_broadcaster = TransformBroadcaster(self)  # Corrected to pass 'self' as the node
        
        # GPIO Pins for encoders
        self.left_a = 5
        self.left_b = 6
        self.right_a = 20
        self.right_b = 21

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.left_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder counts and direction
        self.prev_left_encoder_count = 0
        self.left_encoder_count = 0
        self.prev_right_encoder_count = 0
        self.right_encoder_count = 0
        self.left_direction = 0
        self.right_direction = 0

        # Physical constants
        #circumference = 22cm
        #ticks per rev = 660
        self.wheel_radius = 0.035
        self.track_width = 0.2 
        self.ticks_per_meter = 3000  

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Attach interrupts
        GPIO.add_event_detect(self.left_a, GPIO.BOTH, callback=self.update_left_encoder)
        GPIO.add_event_detect(self.right_a, GPIO.BOTH, callback=self.update_right_encoder)

        self.create_timer(0.2, self.timer_callback)

    def update_left_encoder(self, channel):
        state_a = GPIO.input(self.left_a)
        state_b = GPIO.input(self.left_b)
        if state_a == state_b:
            self.left_direction = 1
        else:
            self.left_direction = -1
        self.left_encoder_count += 1

    def update_right_encoder(self, channel):
        state_a = GPIO.input(self.right_a)
        state_b = GPIO.input(self.right_b)
        if state_a == state_b:
            self.right_direction = 1
        else:
            self.right_direction = -1
        self.right_encoder_count += 1

    def timer_callback(self):
        current_time = self.get_clock().now()

        # Calculate the distance each wheel has traveled
        left_distance = ((self.left_encoder_count - self.prev_left_encoder_count) / self.ticks_per_meter) * self.left_direction
        right_distance = ((self.right_encoder_count - self.prev_right_encoder_count) / self.ticks_per_meter) * self.right_direction
        self.get_logger().info(f"LEFT {self.left_encoder_count } {self.prev_left_encoder_count }")
        self.get_logger().info(f"RIGHT {self.right_encoder_count } {self.prev_right_encoder_count }")
        # Reset encoder counts
        self.prev_left_encoder_count = self.left_encoder_count 
        self.prev_right_encoder_count = self.right_encoder_count 

        # Calculate joint positions (assuming wheel radius is involved in conversion from distance to angle)
        left_joint_position = (self.left_encoder_count / self.ticks_per_meter) * self.left_direction / self.wheel_radius
        right_joint_position = (self.right_encoder_count / self.ticks_per_meter) * self.right_direction / self.wheel_radius

        # Calculate the average forward movement and orientation change
        delta_x = (left_distance + right_distance) / 2.0 * math.cos(self.theta)
        delta_y = (left_distance + right_distance) / 2.0 * math.sin(self.theta)
        delta_theta = (right_distance - left_distance) / self.track_width

        # Update pose estimation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize theta

        # Create quaternion from yaw
        q = quaternion_from_euler(0, 0, self.theta)
        odom_quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Broadcast the transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat
        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        self.odometry_publisher.publish(odom)

        # Publish the joint state message
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_joint_position, right_joint_position]
        self.joint_state_publisher.publish(joint_state)

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
