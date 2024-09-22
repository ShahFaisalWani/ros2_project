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
                    FindPackageShare("diffdrive_gpio"),
                    "launch",
                    "rsp.launch.py",
                ]
            )])
        )

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("diffdrive_gpio"), "urdf", "diffbot.urdf.xacro"]
    #         ),
    #         " "
    #     ]
    # )
    
    # robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}


    controller_params_file = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_gpio"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # ROS2 control node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
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

    # # Joint state broadcaster spawner
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    # # Robot controller spawner
    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    # # Delay the start of joint_state_broadcaster after robot_controller
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    # # Add a timer to delay the robot controller spawner to ensure controller_manager is fully ready
    # delayed_robot_controller_spawner = TimerAction(
    #     period=3.0,  # Delay for 5 seconds (adjust if necessary)
    #     actions=[robot_controller_spawner]
    # )

    # Sequence of nodes
    nodes = [
        rsp,
        # control_node,
        # robot_state_pub_node,
        # delayed_robot_controller_spawner,  # Add the delay here
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ]

    return LaunchDescription(nodes)
