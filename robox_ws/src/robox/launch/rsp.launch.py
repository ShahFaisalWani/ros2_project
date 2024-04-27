from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('robox'))
    xacro_file = os.path.join(pkg_path,'description','robox.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # node_static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'odom'],
    #     output='screen'
    # )

    return LaunchDescription([
        node_robot_state_publisher,
        # node_static_transform_publisher
    ])