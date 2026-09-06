#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share_dir = get_package_share_directory("ayg_slam_ros2")
    dataset_dir = "/path/to/dataset"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share_dir, "launch", "ayg_slam.launch.py")
                ),
                launch_arguments={
                    "dataset_dir": dataset_dir,
                    "association_file": os.path.join(dataset_dir, "associations.txt"),
                    "settings_file": os.path.join(
                        share_dir, "Examples", "RGB-D", "airsim_new.yaml"
                    ),
                    "rviz_config": os.path.join(
                        share_dir,
                        "Examples",
                        "RGB-D",
                        "octomap_view_airsim.rviz",
                    ),
                    "octomap_resolution": "1.0",
                    "octomap_max_range": "100.0",
                }.items(),
            )
        ]
    )
