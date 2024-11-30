import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0')

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb serial port to connected lidar'),

        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',            
        #     output='screen',
        #     parameters=[{
        #         'serial_port': serial_port,
        #         # 'serial_baudrate': 230400,
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame', 
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard'
        #         }]
        #     ),
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': serial_port, 'frame_id': 'laser_frame'}],
            output='screen'),
    ])
