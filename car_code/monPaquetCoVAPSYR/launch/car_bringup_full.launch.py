#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('monPaquetCoVAPSYR')

    car_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_dir,
                'launch',
                'car_bringup.launch.py'
            )
        )
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_dir,
                'launch',
                'camera.launch.py'
            )
        )
    )

    return LaunchDescription([
        car_bringup,
        camera,
    ])
