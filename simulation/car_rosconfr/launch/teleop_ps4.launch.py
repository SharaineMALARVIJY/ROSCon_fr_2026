from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    ps4_teleop = Node(
        package='car_rosconfr',
        executable='ps4_teleop_ackermann',
        name='ps4_teleop_ackermann',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        ps4_teleop,
    ])
