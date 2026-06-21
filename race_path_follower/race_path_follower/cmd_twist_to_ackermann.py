#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool


class SimpleAckermannMux(Node):
    """
    Convertisseur + mux de commandes pour yellow_car.

    Entrées:
      - /cmd_twist : commande normale Nav2 en geometry_msgs/Twist
      - /yellow_car/cmd_ackermann_recovery : commande recovery LiDAR en AckermannDrive
      - /yellow_car/recovery_enable : activation de la recovery LiDAR en std_msgs/Bool
      - /yellow_car/cmd_ackermann_direction_recovery : commande recovery mauvais sens en AckermannDrive
      - /yellow_car/direction_recovery_enable : activation de la recovery mauvais sens en std_msgs/Bool
      - /yellow_car/cmd_ackermann_teleop : commande téléop prioritaire en AckermannDrive

    Sortie:
      - /yellow_car/cmd_ackermann : commande finale pour la voiture

    Priorité:
      1) téléop si commande non nulle
      2) recovery mauvais sens si direction_recovery_enable == true
      3) recovery LiDAR si recovery_enable == true
      4) Nav2 sinon
      5) STOP si aucune commande Nav2 n'a encore été reçue

    Important:
      - La téléop rend la main en publiant une commande Ackermann nulle.
      - Les recoveries rendent la main avec leur topic enable à false.
      - Les recoveries peuvent publier speed=0 et steering_angle=0 tout en gardant le contrôle
        si leur enable reste true.
      - Il n'y a pas de logique de timeout: le mux utilise simplement le dernier état reçu.
    """

    def __init__(self):
        super().__init__('simple_ackermann_mux')

        self.declare_parameter('cmd_twist_topic', '/cmd_twist')
        self.declare_parameter('teleop_topic', '/yellow_car/cmd_ackermann_teleop')
        self.declare_parameter('direction_recovery_topic', '/yellow_car/cmd_ackermann_direction_recovery')
        self.declare_parameter('direction_recovery_enable_topic', '/yellow_car/direction_recovery_enable')
        self.declare_parameter('recovery_topic', '/yellow_car/cmd_ackermann_recovery')
        self.declare_parameter('recovery_enable_topic', '/yellow_car/recovery_enable')
        self.declare_parameter('output_topic', '/yellow_car/cmd_ackermann')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('wheelbase', 0.257)

        self.declare_parameter('teleop_speed_deadband', 0.001)
        self.declare_parameter('teleop_steering_deadband', 0.001)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.teleop_speed_deadband = float(self.get_parameter('teleop_speed_deadband').value)
        self.teleop_steering_deadband = float(self.get_parameter('teleop_steering_deadband').value)

        self.nav2_cmd = None

        self.teleop_cmd = AckermannDrive()

        self.direction_recovery_cmd = AckermannDrive()
        self.direction_recovery_enabled = False

        self.recovery_cmd = AckermannDrive()
        self.recovery_enabled = False

        self.pub = self.create_publisher(
            AckermannDrive,
            str(self.get_parameter('output_topic').value),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_twist_topic').value),
            self.nav2_callback,
            10,
        )

        self.create_subscription(
            AckermannDrive,
            str(self.get_parameter('teleop_topic').value),
            self.teleop_callback,
            10,
        )

        self.create_subscription(
            AckermannDrive,
            str(self.get_parameter('direction_recovery_topic').value),
            self.direction_recovery_callback,
            10,
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter('direction_recovery_enable_topic').value),
            self.direction_recovery_enable_callback,
            10,
        )

        self.create_subscription(
            AckermannDrive,
            str(self.get_parameter('recovery_topic').value),
            self.recovery_callback,
            10,
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter('recovery_enable_topic').value),
            self.recovery_enable_callback,
            10,
        )

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self.timer_callback)

    def teleop_is_active(self):
        return (
            abs(float(self.teleop_cmd.speed)) > self.teleop_speed_deadband
            or abs(float(self.teleop_cmd.steering_angle)) > self.teleop_steering_deadband
        )

    def twist_to_ackermann(self, msg):
        v = float(msg.linear.x)
        wz = float(msg.angular.z)

        cmd = AckermannDrive()
        cmd.speed = v

        if abs(v) > 1e-6:
            cmd.steering_angle = math.atan(self.wheelbase * wz / v)
        else:
            cmd.steering_angle = 0.0

        return cmd

    def select_command(self):
        if self.teleop_is_active():
            return self.teleop_cmd

        if self.direction_recovery_enabled:
            return self.direction_recovery_cmd

        if self.recovery_enabled:
            return self.recovery_cmd

        if self.nav2_cmd is not None:
            return self.nav2_cmd

        return AckermannDrive()

    def timer_callback(self):
        self.pub.publish(self.select_command())

    def nav2_callback(self, msg):
        self.nav2_cmd = self.twist_to_ackermann(msg)

    def teleop_callback(self, msg):
        self.teleop_cmd = msg

    def direction_recovery_callback(self, msg):
        self.direction_recovery_cmd = msg

    def direction_recovery_enable_callback(self, msg):
        self.direction_recovery_enabled = bool(msg.data)

    def recovery_callback(self, msg):
        self.recovery_cmd = msg

    def recovery_enable_callback(self, msg):
        self.recovery_enabled = bool(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAckermannMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(AckermannDrive())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
