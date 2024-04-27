from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='robox',
        #     executable='lidar_node',
        #     output='screen',
        # ),
        # Node(
        #     package='tf2_ros',
        #     name="laser_frame_to_map",
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'laser_frame'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     name="base_link_to_odom",
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
        # ),
         Node(
            package='robox',
            executable='drive_node',
            output='screen',
        ),
         Node(
            package='robox',
            executable='encoder_node',
            output='screen',
        ),
    ])