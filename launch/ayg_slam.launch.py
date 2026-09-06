#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "ayg_slam_ros2"


def _is_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_slam(context):
    dataset_dir = os.path.expanduser(
        LaunchConfiguration("dataset_dir").perform(context)
    )
    association_file = os.path.expanduser(
        LaunchConfiguration("association_file").perform(context)
    )
    settings_file = os.path.expanduser(
        LaunchConfiguration("settings_file").perform(context)
    )
    vocabulary_file = os.path.expanduser(
        LaunchConfiguration("vocabulary_file").perform(context)
    )
    output_dir = os.path.expanduser(
        LaunchConfiguration("output_dir").perform(context)
    )

    required_paths = {
        "dataset_dir": dataset_dir,
        "association_file": association_file,
        "settings_file": settings_file,
        "vocabulary_file": vocabulary_file,
    }
    missing = [name for name, path in required_paths.items() if not os.path.exists(path)]
    if missing:
        details = ", ".join(f"{name}={required_paths[name]}" for name in missing)
        raise RuntimeError(f"AYG-SLAM launch path does not exist: {details}")

    os.makedirs(output_dir, exist_ok=True)
    executable = os.path.join(
        get_package_prefix(PACKAGE_NAME),
        "lib",
        PACKAGE_NAME,
        "rgbd_tum_octomap",
    )
    command = [
        executable,
        vocabulary_file,
        settings_file,
        dataset_dir,
        association_file,
    ]
    process = ExecuteProcess(
        cmd=command,
        cwd=output_dir,
        output="screen",
        emulate_tty=True,
    )

    if _is_true(LaunchConfiguration("use_octomap").perform(context)):
        return [TimerAction(period=2.0, actions=[process])]
    return [process]


def generate_launch_description():
    share_dir = get_package_share_directory(PACKAGE_NAME)
    default_dataset = "/path/to/dataset"

    arguments = [
        DeclareLaunchArgument(
            "dataset_dir",
            default_value=default_dataset,
            description="TUM/AirSim RGB-D sequence root directory",
        ),
        DeclareLaunchArgument(
            "association_file",
            default_value=os.path.join(default_dataset, "associate_ours.txt"),
            description="RGB/depth association text file",
        ),
        DeclareLaunchArgument(
            "settings_file",
            default_value=os.path.join(share_dir, "Examples", "RGB-D", "TUM3.yaml"),
            description="AYG-SLAM OpenCV YAML settings file",
        ),
        DeclareLaunchArgument(
            "vocabulary_file",
            default_value=os.path.join(share_dir, "Vocabulary", "ORBvoc.txt"),
            description="ORB-SLAM2 text vocabulary",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                share_dir, "Examples", "RGB-D", "octomap_view_tum.rviz"
            ),
            description="RViz2 configuration file",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.path.join(os.path.expanduser("~"), ".ros", "ayg_slam"),
            description="Directory for CameraTrajectory.txt and KeyFrameTrajectory.txt",
        ),
        DeclareLaunchArgument("use_octomap", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("octomap_resolution", default_value="0.05"),
        DeclareLaunchArgument("octomap_max_range", default_value="8.0"),
    ]

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_octomap")),
        remappings=[("cloud_in", "/AYG/Local_Point_Clouds")],
        parameters=[
            {
                "frame_id": "AYG/map",
                "resolution": ParameterValue(
                    LaunchConfiguration("octomap_resolution"), value_type=float
                ),
                "sensor_model.max_range": ParameterValue(
                    LaunchConfiguration("octomap_max_range"), value_type=float
                ),
                "filter_ground": False,
                "filter_speckles": True,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(arguments + [octomap_server, rviz, OpaqueFunction(function=_launch_slam)])
