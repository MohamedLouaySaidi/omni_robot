import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("omni_robot_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    sensors = os.path.join(bringup_share, "config", "sensors.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_ultrasonic.yaml")
    watchdog = os.path.join(bringup_share, "config", "slam_watchdog.yaml")

    slam_launch = os.path.join(slam_share, "launch", "online_async_launch.py")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
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
                package="omni_robot_hardware",
                executable="ultrasonic_perception_node",
                name="ultrasonic_perception_node",
                parameters=[{"publish_rate_hz": 5.0}],
                remappings=[("scan", "scan_ultra")],
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
                executable="slam_watchdog_node",
                name="slam_watchdog_node",
                parameters=[watchdog],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                }.items(),
            ),
        ]
    )
