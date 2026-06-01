import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("omni_robot_bringup")
    sensors = os.path.join(pkg_share, "config", "sensors.yaml")
    drive = os.path.join(pkg_share, "config", "drive.yaml")
    bt_params = os.path.join(pkg_share, "config", "bt_reactive_explore_v2.yaml")
    bt_xml = os.path.join(pkg_share, "config", "bt_reactive_explore.xml")

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
                executable="bt_reactive_explorer_v2_node",
                name="bt_reactive_explorer_v2_node",
                parameters=[bt_params, {"bt_xml_file": bt_xml}],
                output="screen",
            ),
        ]
    )
