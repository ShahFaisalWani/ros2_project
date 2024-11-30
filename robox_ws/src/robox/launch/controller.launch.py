from launch import LaunchDescription
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # Import for wrapping params
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
                [
                    FindPackageShare("robox"),
                    "launch",
                    "rsp.launch.py",
                ]
            )])
        )

    controller_params_file = PathJoinSubstitution(
        [
            FindPackageShare("robox"),
            "config",
            "controllers.yaml",
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_diff_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    nodes = [
        rsp,
        delayed_controller_manager,
        delayed_diff_controller_spawner,
        delayed_joint_broad_spawner
    ]

    return LaunchDescription(nodes)
