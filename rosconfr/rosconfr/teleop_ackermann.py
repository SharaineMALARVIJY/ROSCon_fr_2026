#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive


HELP = """
Téléopération voiture Ackermann

Touches AZERTY :
  z : accélérer
  s : ralentir / reculer
  q : tourner à gauche
  d : tourner à droite
  espace : stop vitesse + direction
  a : direction au centre
  x : quitter

Commandes publiées sur /yellow_car/cmd_ackermann
"""


class TeleopAckermann(Node):
    def __init__(self):
        super().__init__('teleop_ackermann')

        self.publisher = self.create_publisher(
            AckermannDrive,
            '/yellow_car/cmd_ackermann',
            10
        )

        self.speed = 0.0
        self.steering = 0.0

        self.max_speed = 3.0
        self.max_reverse_speed = -1.0
        self.max_steering = 0.6

        self.speed_step = 0.2
        self.steering_step = 0.08

        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(HELP)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def handle_key(self, key):
        if key == 'z':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key == 'q':
            self.steering += self.steering_step
        elif key == 'd':
            self.steering -= self.steering_step
        elif key == ' ':
            self.speed = 0.0
            self.steering = 0.0
        elif key == 'a':
            self.steering = 0.0

        self.speed = self.clamp(
            self.speed,
            self.max_reverse_speed,
            self.max_speed
        )
        self.steering = self.clamp(
            self.steering,
            -self.max_steering,
            self.max_steering
        )

        self.get_logger().info(
            f"speed={self.speed:.2f} m/s | steering={self.steering:.2f} rad"
        )

    def publish_command(self):
        msg = AckermannDrive()
        msg.speed = float(self.speed)
        msg.steering_angle = float(self.steering)
        self.publisher.publish(msg)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopAckermann()

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in ('x', '\x03'):
                node.get_logger().info("Arrêt demandé")
                break

            if key:
                node.handle_key(key)

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé avec KeyboardInterrupt")

    finally:
        # Stopper la voiture avant de quitter
        node.speed = 0.0
        node.steering = 0.0
        node.publish_command()

        node.destroy_node()
        rclpy.shutdown()

        # Restaurer le terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
