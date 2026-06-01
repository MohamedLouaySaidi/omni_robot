import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("omni_robot_bringup")
    sensors = os.path.join(pkg_share, "config", "sensors.yaml")
    drive = os.path.join(pkg_share, "config", "drive.yaml")
    wander = os.path.join(pkg_share, "config", "simple_wander.yaml")

    return LaunchDescription(
        [
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
                executable="tank_drive_node",
                name="tank_drive_node",
                parameters=[drive],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="simple_wander_node",
                name="simple_wander_node",
                parameters=[wander],
                output="screen",
            ),
        ]
    )
