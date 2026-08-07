import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("berxel_camera_ros2"),
        "config",
        "berxel_ihawk_100Q.yaml",
    )

    camera1 = Node(
        package="berxel_camera_ros2",
        executable="berxel_camera_ros2",
        namespace="ihawk1",
        name="ihawk1_camera",
        parameters=[
            parameter_file,
            {
                "DeviceName": "ihawk1",
                "serial_number": "HK100QB6513M2B479",
                "device_port": "",
                "device_bus": 0,
                "enable_pointcloud": False,
            },
        ],
        output="screen",
    )

    camera2 = Node(
        package="berxel_camera_ros2",
        executable="berxel_camera_ros2",
        namespace="ihawk2",
        name="ihawk2_camera",
        parameters=[
            parameter_file,
            {
                "DeviceName": "ihawk2",
                "serial_number": "HK100QB5311M2B242",
                "device_port": "",
                "device_bus": 0,
                "enable_pointcloud": False,
            },
        ],
        output="screen",
    )

    return LaunchDescription([camera1, camera2])
