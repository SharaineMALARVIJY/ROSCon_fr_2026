import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            '/usr/local/lib/aarch64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        SetEnvironmentVariable(
            'LIBCAMERA_IPA_MODULE_PATH',
            '/usr/local/lib/aarch64-linux-gnu/libcamera/ipa'
        ),
        SetEnvironmentVariable(
            'LIBCAMERA_RPI_CONFIG_FILE',
            '/usr/local/share/libcamera/pipeline/rpi/pisp/example.yaml'
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            parameters=[
                {'width': 800},
                {'height': 600},
            ],
        ),
    ])
