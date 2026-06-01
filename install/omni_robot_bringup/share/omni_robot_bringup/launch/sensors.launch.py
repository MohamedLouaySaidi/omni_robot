from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("omni_robot_bringup")
    config = os.path.join(pkg_share, "config", "sensors.yaml")

    return LaunchDescription(
        [
            Node(
                package="omni_robot_hardware",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="omni_robot_hardware",
                executable="imu_node",
                name="imu_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
