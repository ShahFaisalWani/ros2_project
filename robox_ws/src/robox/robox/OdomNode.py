import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

import math

class OdomNode(Node):

    def __init__(self):
        super().__init__('ekf_odom_pub')
        
        # Publishers
        self.odom_data_pub = self.create_publisher(Odometry, 'odom_data_euler', 10)
        self.odom_data_pub_quat = self.create_publisher(Odometry, 'odom_data_quat', 10)

        # Subscribers
        self.subscription_right_ticks = self.create_subscription(Int16, 'right_ticks', self.calc_right, 10)
        self.subscription_left_ticks = self.create_subscription(Int16, 'left_ticks', self.calc_left, 10)
        self.subscription_initial_pose = self.create_subscription(PoseStamped, 'initial_2d', self.set_initial_2d, 10)

        # Odometry messages
        self.odom_new = Odometry()
        self.odom_old = Odometry()

        # Initial pose
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.00000000001

        # Constants
        self.ticks_per_revolution = 620
        self.wheel_radius = 0.033
        self.wheel_base = 0.17
        self.ticks_per_meter = 3100

        # Distance calculations
        self.distance_left = 0
        self.distance_right = 0

        # Flag for initial pose
        self.initial_pose_received = False

        # Set the initial pose
        self.odom_old.pose.pose.position.x = self.initial_x
        self.odom_old.pose.pose.position.y = self.initial_y
        self.odom_old.pose.pose.orientation.z = self.initial_theta

        # Loop rate
        self.timer = self.create_timer(0.033, self.timer_callback)

    def set_initial_2d(self, msg):
        self.odom_old.pose.pose.position.x = msg.pose.position.x
        self.odom_old.pose.pose.position.y = msg.pose.position.y
        self.odom_old.pose.pose.orientation.z = msg.pose.orientation.z
        self.initial_pose_received = True

    def calc_left(self, msg):
        # Placeholder for actual tick count calculation
        pass

    def calc_right(self, msg):
        # Placeholder for actual tick count calculation
        pass

    def publish_quat(self):
        q = euler_from_quaternion(0, 0, self.odom_new.pose.pose.orientation.z)

        quat_odom = Odometry()
        quat_odom.header.stamp = self.odom_new.header.stamp
        quat_odom.header.frame_id = 'odom'
        quat_odom.child_frame_id = 'base_link'
        quat_odom.pose.pose.position.x = self.odom_new.pose.pose.position.x
        quat_odom.pose.pose.position.y = self.odom_new.pose.pose.position.y
        quat_odom.pose.pose.orientation.x = q[0]
        quat_odom.pose.pose.orientation.y = q[1]
        quat_odom.pose.pose.orientation.z = q[2]
        quat_odom.pose.pose.orientation.w = q[3]

        self.odom_data_pub_quat.publish(quat_odom)

    def update_odom(self):
      cycle_distance = (self.distance_right + self.distance_left) / 2

      cycle_angle = math.asin((self.distance_right - self.distance_left) / self.wheel_base)

      avg_angle = cycle_angle / 2 + self.odom_old.pose.pose.orientation.z

      if avg_angle > math.pi:
          avg_angle -= 2 * math.pi
      elif avg_angle < -math.pi:
          avg_angle += 2 * math.pi

      self.odom_new.pose.pose.position.x = self.odom_old.pose.pose.position.x + math.cos(avg_angle) * cycle_distance
      self.odom_new.pose.pose.position.y = self.odom_old.pose.pose.position.y + math.sin(avg_angle) * cycle_distance
      self.odom_new.pose.pose.orientation.z = cycle_angle + self.odom_old.pose.pose.orientation.z

      if self.odom_new.pose.pose.orientation.z > math.pi:
          self.odom_new.pose.pose.orientation.z -= 2 * math.pi
      elif self.odom_new.pose.pose.orientation.z < -math.pi:
          self.odom_new.pose.pose.orientation.z += 2 * math.pi

      time_diff = (self.get_clock().now() - self.odom_old.header.stamp).nanoseconds / 1e9
      self.odom_new.twist.twist.linear.x = cycle_distance / time_diff if time_diff > 0 else 0
      self.odom_new.twist.twist.angular.z = cycle_angle / time_diff if time_diff > 0 else 0

      self.odom_new.header.stamp = self.get_clock().now().to_msg()

      self.odom_old.pose.pose.position.x = self.odom_new.pose.pose.position.x
      self.odom_old.pose.pose.position.y = self.odom_new.pose.pose.position.y
      self.odom_old.pose.pose.orientation.z = self.odom_new.pose.pose.orientation.z
      self.odom_old.header.stamp = self.odom_new.header.stamp

      self.odom_data_pub.publish(self.odom_new)

    def timer_callback(self):
        if self.initial_pose_received:
            self.update_odom()
            self.publish_quat()

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomNode()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
