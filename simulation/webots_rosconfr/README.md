# rosconfr

`rosconfr` is a ROS 2 package for running a Webots-based Ackermann vehicle simulation built around the `yellow_car` robot. This package will be used during the Hackathon organized for the ROSCon FR 2026 in Paris, and derived from [the package](https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay/tree/main/Simulator) used during the [CoVAPSy challenge](https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/).

The package provides:

- two launch files for the standard and camera-enabled simulations,
- a Webots driver plugin that applies Ackermann commands to the vehicle,
- a joint state publisher for RViz / TF-based visualization,
- the robot description, worlds, and Webots PROTO files used by the simulation.

## Launch files

### `webots_rosconfr.launch.py`

Starts the standard simulation without the front camera.

```bash
ros2 launch rosconfr webots_rosconfr.launch.py
```

### `webots_rosconfr_cam.launch.py`

Starts the simulation with the front camera enabled.

```bash
ros2 launch rosconfr webots_rosconfr_cam.launch.py
```

## Nodes and processes

Both launch files start the same core ROS graph, with camera topics added by the camera variant.

| Name | Kind | Source | Role |
|---|---|---|---|
| `webots` | process | `WebotsLauncher` | Starts the Webots simulator with the selected world. |
| `webots_bridge` | node | `webots_ros2_driver::WebotsController` | Connects the `yellow_car` Webots robot to ROS 2 and exposes sensor interfaces declared in the URDF. |
| `robot_state_publisher` | node | `robot_state_publisher` | Publishes the TF tree from the robot URDF. |
| `yellow_driver` | node created inside the Webots plugin | `rosconfr.yellow_driver.YellowDriver` | Subscribes to `/yellow_car/cmd_ackermann`, applies speed and steering commands to the simulated car, and publishes measured wheel/joint states plus current speed. |

## Topics

### Common topics

These topics are available in both launch files.

| Topic | Type | Direction | Provided by | Notes |
|---|---|---|---|---|
| `/yellow_car/cmd_ackermann` | `ackermann_msgs/msg/AckermannDrive` | subscribed | `yellow_driver` | Main control input for the simulated vehicle. |
| `/yellow_car/scan` | `sensor_msgs/msg/LaserScan` | published | `webots_bridge` | Lidar output from the `RpLidarA2` device. |
| `/joint_states` | `sensor_msgs/msg/JointState` | published | `yellow_driver` | Steering and wheel rotation states measured from the simulated vehicle joints. |
| `/yellow_car/current_speed` | `std_msgs/msg/Float32` | published | `yellow_driver` | Current vehicle speed measured by Webots and converted to m/s. |
| `/tf` | `tf2_msgs/msg/TFMessage` | published | `robot_state_publisher` | Dynamic transforms for the robot model. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | published | `robot_state_publisher` | Static transforms from the robot URDF. |

### Additional topics in `webots_rosconfr_cam.launch.py`

These topics are only available in the camera-enabled launch file.

| Topic | Type | Direction | Provided by | Notes |
|---|---|---|---|---|
| `/yellow_car/image_raw` | `sensor_msgs/msg/Image` | published | `webots_bridge` | Front camera image stream. |
| `/yellow_car/camera_info` | `sensor_msgs/msg/CameraInfo` | published | `webots_bridge` | Camera calibration and projection metadata. |

## Notes

- `yellow_driver` now reads Webots position sensors to publish `/joint_states`, and reads `getCurrentSpeed()` to publish `/yellow_car/current_speed` in m/s.
- Steering is derived from the `AckermannDrive.steering_angle` field and mirrored to both front steering joints.
- The camera is only declared in `resource/TT02_jaune_cam.urdf`, so it is only exposed by the camera launch file.
