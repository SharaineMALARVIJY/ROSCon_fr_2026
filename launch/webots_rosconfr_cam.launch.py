import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('rosconfr')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'TT02_jaune_cam.urdf')
    with open(robot_description_path, 'r') as file:
        robot_description = file.read()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds',
                           'Piste_CoVAPSy_2025a_camera.wbt'),
        mode='realtime',
        gui=True
    )

    webots_bridge = WebotsController(
        robot_name='yellow_car',  # must match the robot name in world file
        parameters=[{'robot_description': robot_description_path},],
        remappings=[
            ('__node', 'webots_bridge'),  # Change node name
        ],
        respawn=True,  # Respawn robot after simulation reset in Webots
        # respawn_delay=1.0
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # CoVAPSy_conduite = Node(
    #     package='monPaquetCoVAPSy',
    #     executable='CoVAPSy_conduite',
    # )

    return LaunchDescription([
        webots,
        webots_bridge,
        robot_state_publisher,
        # CoVAPSy_conduite,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
