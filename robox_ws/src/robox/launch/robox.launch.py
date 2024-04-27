import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
      IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robox'),'launch/slam.launch.yaml')
        )
      )
    ])