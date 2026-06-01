from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("omni_robot_bringup")
    sensors = os.path.join(pkg_share, "config", "sensors.yaml")
    drive = os.path.join(pkg_share, "config", "drive.yaml")
    avoidance = os.path.join(pkg_share, "config", "avoidance.yaml")

    return LaunchDescription(
        [
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
                executable="obstacle_avoider_node",
                name="obstacle_avoider_node",
                parameters=[avoidance],
                output="screen",
            ),
        ]
    )
