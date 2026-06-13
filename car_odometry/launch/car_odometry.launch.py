from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('car_odometry'),
        'config',
        'car_odometry.yaml'
    )

    tf_base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_footprint_to_base_link",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    return LaunchDescription([
        tf_base_footprint_to_base_link,
        Node(
            package='car_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('odometry/filtered', 'odom'),
            ],
        ),
    ])