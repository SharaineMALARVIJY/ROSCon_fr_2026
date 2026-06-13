import rclpy
import math

from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class YellowDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep_ms = int(self.__robot.getBasicTimeStep())
        self.__vitesse_m_s = 0.0
        self.__steering_angle_rad = 0.0
        self.__is_active = True
        self.__joint_state_names = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        self.__front_left_steering_sensor = self.__robot.getDevice('left_steer_sensor')
        self.__front_right_steering_sensor = self.__robot.getDevice('right_steer_sensor')
        self.__front_left_wheel_sensor = self.__robot.getDevice('left_front_sensor')
        self.__front_right_wheel_sensor = self.__robot.getDevice('right_front_sensor')
        self.__rear_left_wheel_sensor = self.__robot.getDevice('left_rear_sensor')
        self.__rear_right_wheel_sensor = self.__robot.getDevice('right_rear_sensor')

        self.__front_left_steering_sensor.enable(self.__timestep_ms)
        self.__front_right_steering_sensor.enable(self.__timestep_ms)
        self.__front_left_wheel_sensor.enable(self.__timestep_ms)
        self.__front_right_wheel_sensor.enable(self.__timestep_ms)
        self.__rear_left_wheel_sensor.enable(self.__timestep_ms)
        self.__rear_right_wheel_sensor.enable(self.__timestep_ms)

        self.__accelerometer = self.__robot.getDevice('accelerometer')
        self.__imu_orientation = self.__robot.getDevice('inertial_unit')
        self.__gyro = self.__robot.getDevice('gyro')

        self.__accelerometer.enable(self.__timestep_ms)
        self.__imu_orientation.enable(self.__timestep_ms)
        self.__gyro.enable(self.__timestep_ms)

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('yellow_driver')
        self.__node.create_subscription(AckermannDrive,
            '/yellow_car/cmd_ackermann',
            self.__cmd_ackermann_callback,
            1,
        )
        self.__joint_state_publisher = self.__node.create_publisher(
            JointState,
            '/yellow_car/joint_states',
            10,
        )
        self.__current_speed_publisher = self.__node.create_publisher(
            Float32,
            '/yellow_car/current_speed',
            10,
        )
        self.__imu_publisher = self.__node.create_publisher(
            Imu,
            '/yellow_car/imu',
            10,
        )
        self.__node.get_logger().info("Noeud créé")
        self.__robot.setCruisingSpeed(0.0) # En km/h
        self.__robot.setSteeringAngle(0.0) # En radian

    def __cmd_ackermann_callback(self, message):
        self.__vitesse_m_s = message.speed
        self.__steering_angle_rad = message.steering_angle
        # self.__node.get_logger().info(
        #     f"[yellow_driver] Reçu : vitesse = {self.__vitesse_m_s} m/s, braquage = {self.__steering_angle_rad} rad")

    def __publish_joint_states(self):
        message = JointState()
        message.header.stamp = self.__node.get_clock().now().to_msg()
        message.name = self.__joint_state_names
        message.position = [
            -self.__front_left_steering_sensor.getValue(),
            -self.__front_right_steering_sensor.getValue(),
            self.__front_left_wheel_sensor.getValue(),
            self.__front_right_wheel_sensor.getValue(),
            self.__rear_left_wheel_sensor.getValue(),
            self.__rear_right_wheel_sensor.getValue(),
        ]
        self.__joint_state_publisher.publish(message)

    def __publish_current_speed(self):
        message = Float32()
        # message.data = self.__robot.getCurrentSpeed() / 3.6
        message.data = self.__vitesse_m_s
        self.__current_speed_publisher.publish(message)

    def __publish_imu(self):
        roll, pitch, yaw = self.__imu_orientation.getRollPitchYaw()
        gx, gy, gz = self.__gyro.getValues()
        ax, ay, az = self.__accelerometer.getValues()

        msg = Imu()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # RPY -> quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self.__imu_publisher.publish(msg)

    def _shutdown_ros(self):
        if not self.__is_active:
            return
        self.__is_active = False
        self.__robot.setCruisingSpeed(0.0)
        self.__robot.setSteeringAngle(0.0)
        if hasattr(self, '_YellowDriver__node') and self.__node is not None:
            self.__node.destroy_node()
            self.__node = None
        if rclpy.ok():
            rclpy.shutdown()

    def step(self):
        if not self.__is_active or not rclpy.ok():
            self._shutdown_ros()
            return
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self._shutdown_ros()
            return
        self.__robot.setCruisingSpeed(self.__vitesse_m_s*3.6) # Conversion m/s -> km/h
        self.__robot.setSteeringAngle(-self.__steering_angle_rad)
        self.__publish_joint_states()
        self.__publish_current_speed()
        self.__publish_imu()

    def __del__(self):
        self._shutdown_ros()
