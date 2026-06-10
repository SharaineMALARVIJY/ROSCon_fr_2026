#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Webots simulation time"
    )

    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Odometry topic from simulation"
    )

    declare_scan_topic = DeclareLaunchArgument(
        "scan_topic",
        default_value="/blue_car/scan",
        description="LaserScan topic from blue car"
    )

    package_dir = get_package_share_directory("rosconfr")
    urdf_path = os.path.join(package_dir, "resource", "TT02_bleu.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Publie les TF statiques/dynamiques du robot :
    # base_link -> laser_link
    # base_link -> roues, etc.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="blue_robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Transform base_footprint -> base_link.
    # Utile pour beaucoup de stacks Nav2/SLAM.
    # Si tu n'utilises que base_link, ce n'est pas gênant.
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

    # EKF très simple : prend /odom et publie le TF odom -> base_link.
    # Il ne fait pas de localisation globale, il ne charge pas de carte.
    ekf_odom_to_tf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frequency": 30.0,
                "two_d_mode": True,
                "publish_tf": True,

                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "odom",

                "odom0": odom_topic,
                "odom0_queue_size": 10,
                "odom0_differential": False,
                "odom0_relative": False,

                # x, y, z,
                # roll, pitch, yaw,
                # vx, vy, vz,
                # vroll, vpitch, vyaw,
                # ax, ay, az
                "odom0_config": [
                    True,  True,  False,
                    False, False, True,
                    True,  False, False,
                    False, False, True,
                    False, False, False,
                ],
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_topic,
        declare_scan_topic,

        robot_state_publisher,
        tf_base_footprint_to_base_link,
        # ekf_odom_to_tf,
    ])
