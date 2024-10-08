# This file is a launch file that launch all the package at the same time
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch from the first package
    mpv_package = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mpv'), 'launch', 'mpv_launch.py')])
    )

    # Launch from the second package
    station_package = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('station'), 'launch', 'station_launch.py')])
    )

    # Return both launch descriptions
    return LaunchDescription([
        mpv_package,
        station_package
    ])
