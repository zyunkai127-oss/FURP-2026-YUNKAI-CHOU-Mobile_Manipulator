from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("eye_to_hand_grasp_demo"),
        "config",
        "grasp_demo.yaml",
    )

    # T_panda_link0_camera: camera at [0.9, -0.4, 0.9], looking at the object.
    fixed_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fixed_eye_to_hand_camera_tf",
        output="screen",
        arguments=[
            "--x", "0.9", "--y", "-0.4", "--z", "0.9",
            "--qx", "0.894484492", "--qy", "0.270828113",
            "--qz", "-0.103087275", "--qw", "-0.340474140",
            "--frame-id", "panda_link0",
            "--child-frame-id", "camera_color_optical_frame",
        ],
    )

    grasp_demo = Node(
        package="eye_to_hand_grasp_demo",
        executable="grasp_demo",
        name="eye_to_hand_grasp_demo",
        output="screen",
        parameters=[
            config,
            {
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description="Plan and execute motions; false performs TF and IK only",
            ),
            fixed_camera_tf,
            grasp_demo,
        ]
    )
