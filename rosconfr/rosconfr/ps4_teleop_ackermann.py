#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive


class PS4TeleopAckermann(Node):
    def __init__(self):
        super().__init__('ps4_teleop_ackermann')

        # Topics
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_topic', '/blue_car/cmd_ackermann')

        # Mapping manette PS4 probable avec joy_node Linux
        # Stick gauche vertical = vitesse
        # Stick droit horizontal = direction
        self.declare_parameter('axis_throttle', 1)
        self.declare_parameter('axis_steering', 3)

        # Bouton stop optionnel
        self.declare_parameter('button_stop', 1)  # rond souvent = 1

        # Limites voiture
        self.declare_parameter('max_speed', 1.6)
        self.declare_parameter('max_reverse_speed', 0.8)
        self.declare_parameter('max_steering_angle', 0.28)
        self.declare_parameter('deadzone', 0.08)

        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.axis_throttle = int(self.get_parameter('axis_throttle').value)
        self.axis_steering = int(self.get_parameter('axis_steering').value)

        self.button_stop = int(self.get_parameter('button_stop').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        self.last_cmd = AckermannDrive()
        self.last_joy_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(
            AckermannDrive,
            self.cmd_topic,
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10
        )

        # Publie régulièrement la dernière commande
        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info('PS4 teleop Ackermann démarré')
        self.get_logger().info(f'Lecture manette : {self.joy_topic}')
        self.get_logger().info(f'Commande voiture : {self.cmd_topic}')
        self.get_logger().info('Stick gauche vertical = avant/arrière')
        self.get_logger().info('Stick droit horizontal = direction')
        self.get_logger().info('Rond = stop')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def get_axis(self, joy_msg, index):
        if index < 0 or index >= len(joy_msg.axes):
            return 0.0
        return float(joy_msg.axes[index])

    def get_button(self, joy_msg, index):
        if index < 0 or index >= len(joy_msg.buttons):
            return 0
        return int(joy_msg.buttons[index])

    def joy_callback(self, joy_msg):
        self.last_joy_time = self.get_clock().now()

        stop = self.get_button(joy_msg, self.button_stop)

        if stop == 1:
            self.last_cmd.speed = 0.0
            self.last_cmd.steering_angle = 0.0
            return

        throttle_axis = self.apply_deadzone(
            self.get_axis(joy_msg, self.axis_throttle)
        )

        steering_axis = self.apply_deadzone(
            self.get_axis(joy_msg, self.axis_steering)
        )

        # Inversion avant/arrière corrigée :
        # si stick gauche vers le haut donne +1, on garde +.
        # si chez toi c'est encore inversé, remplace par: throttle = -throttle_axis
        throttle = throttle_axis

        if throttle >= 0.0:
            speed = throttle * self.max_speed
        else:
            speed = throttle * self.max_reverse_speed

        # Stick droit horizontal pour direction
        steering = steering_axis * self.max_steering_angle

        self.last_cmd.speed = float(speed)
        self.last_cmd.steering_angle = float(steering)

    def publish_cmd(self):
        # Sécurité : si plus de message /joy depuis 0.5 s, stop
        now = self.get_clock().now()
        dt = (now - self.last_joy_time).nanoseconds * 1e-9

        msg = AckermannDrive()

        if dt > 0.5:
            msg.speed = 0.0
            msg.steering_angle = 0.0
        else:
            msg.speed = self.last_cmd.speed
            msg.steering_angle = self.last_cmd.steering_angle

        self.cmd_pub.publish(msg)

    def stop_car(self):
        msg = AckermannDrive()
        msg.speed = 0.0
        msg.steering_angle = 0.0
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PS4TeleopAckermann()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
