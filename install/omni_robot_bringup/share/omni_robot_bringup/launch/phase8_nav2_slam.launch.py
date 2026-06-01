import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("omni_robot_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    sensors = os.path.join(bringup_share, "config", "sensors.yaml")
    drive = os.path.join(bringup_share, "config", "drive.yaml")
    avoidance = os.path.join(bringup_share, "config", "avoidance_nav.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params_ultrasonic.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_ultrasonic.yaml")

    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    slam_launch = os.path.join(slam_share, "launch", "online_async_launch.py")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            Node(
                package="omni_robot_hardware",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                parameters=[sensors],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="imu_node",
                name="imu_node",
                parameters=[sensors],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_imu",
                arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_front_us",
                arguments=[
                    "0.17",
                    "0",
                    "0.05",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "front_ultrasonic_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_back_us",
                arguments=[
                    "-0.17",
                    "0",
                    "0.05",
                    "3.14159",
                    "0",
                    "0",
                    "base_link",
                    "back_ultrasonic_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_left_us",
                arguments=[
                    "0",
                    "0.12",
                    "0.05",
                    "0",
                    "0",
                    "1.5707",
                    "base_link",
                    "left_ultrasonic_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_right_us",
                arguments=[
                    "0",
                    "-0.12",
                    "0.05",
                    "0",
                    "0",
                    "-1.5707",
                    "base_link",
                    "right_ultrasonic_link",
                ],
            ),
            Node(
                package="omni_robot_hardware",
                executable="ultrasonic_perception_node",
                name="ultrasonic_perception_node",
                parameters=[{"publish_rate_hz": 5.0}],
                remappings=[("scan", "scan_ultra")],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="obstacle_avoider_node",
                name="obstacle_avoider_node",
                parameters=[avoidance],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="tank_drive_node",
                name="tank_drive_node",
                parameters=[drive],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="open_loop_odom_node",
                name="open_loop_odom_node",
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="nav2_goal_sender_node",
                name="nav2_goal_sender_node",
                parameters=[{"goal_frame": "map", "log_debug": True}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                }.items(),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[nav2_params],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            {"use_sim_time": False},
                            {"autostart": True},
                            {"node_names": nav2_lifecycle_nodes},
                        ],
                    )
                ],
            ),
        ]
    )
