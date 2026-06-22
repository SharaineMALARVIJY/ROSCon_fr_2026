"""Keyboard teleoperation node for the ROSConFR Ackermann car."""

import threading

from ackermann_msgs.msg import AckermannDrive
import click
import rclpy
from rclpy.node import Node


class CarTeleopNode(Node):
    """Publish Ackermann commands from keyboard input."""

    def __init__(self):
        """Initialize parameters, publisher, and keyboard mapping."""
        super().__init__('car_teleop_node')

        self.declare_parameter('topic_name', '/yellow_car/cmd_ackermann')
        self.declare_parameter('linear_speed_increment', 1.0)
        self.declare_parameter('steering_angle_increment', 0.35)

        self._topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value
        self._linear_speed_increment = self.get_parameter(
            'linear_speed_increment').get_parameter_value().double_value
        self._steering_angle_increment = self.get_parameter(
            'steering_angle_increment').get_parameter_value().double_value

        self._publisher = self.create_publisher(
            AckermannDrive,
            self._topic_name,
            10,
        )
        self._keycode = {
            '\x1b[A': 'up',
            '\x1b[B': 'down',
            '\x1b[C': 'right',
            '\x1b[D': 'left',
            's': 'stop',
            'q': 'quit',
        }
        self._current_speed = 0.0
        self._current_steering_angle = 0.0

    def _publish_command(self):
        """Publish the current Ackermann command."""
        message = AckermannDrive()
        message.speed = self._current_speed
        message.steering_angle = self._current_steering_angle
        self._publisher.publish(message)

    def _stop_vehicle(self):
        """Stop the vehicle and center the steering."""
        self._current_speed = 0.0
        self._current_steering_angle = 0.0
        self._publish_command()

    def run(self):
        """Read keyboard input and publish matching commands."""
        self.get_logger().info('Lecture des commandes clavier')
        self.get_logger().info('---------------------------')
        self.get_logger().info(
            "Utilisez les flèches pour téléopérer la voiture, 's' pour "
            "arrêter, 'q' pour quitter."
        )
        self.get_logger().info("Topic utilisé: '%s'" % self._topic_name)
        self.get_logger().info(
            'Incrément vitesse linéaire: %.3f m/s' %
            self._linear_speed_increment
        )
        self.get_logger().info(
            'Incrément braquage: %.3f rad' %
            self._steering_angle_increment
        )
        self.get_logger().info('---------------------------')

        try:
            while True:
                mykey = click.getchar()
                if mykey not in self._keycode:
                    continue

                action = self._keycode[mykey]
                if action == 'up':
                    self._current_speed += self._linear_speed_increment
                elif action == 'down':
                    self._current_speed -= self._linear_speed_increment
                elif action == 'left':
                    self._current_steering_angle += (
                        self._steering_angle_increment
                    )
                elif action == 'right':
                    self._current_steering_angle -= (
                        self._steering_angle_increment
                    )
                elif action == 'stop':
                    self._stop_vehicle()
                    self.get_logger().info('Key pressed: "stop"')
                    continue
                elif action == 'quit':
                    self.get_logger().info('Key pressed: "quit"')
                    break

                self.get_logger().info(
                    'Key pressed: "%s" | speed=%.3f m/s | steering=%.3f rad' % (
                        action,
                        self._current_speed,
                        self._current_steering_angle,
                    )
                )
                self._publish_command()
        except (KeyboardInterrupt, EOFError, click.Abort):
            self.get_logger().info('Arrêt demandé depuis le clavier.')
        finally:
            self._stop_vehicle()


def main(args=None):
    """Run the keyboard teleoperation node."""
    rclpy.init(args=args)
    teleop_node = CarTeleopNode()

    thread = threading.Thread(
        target=rclpy.spin,
        args=(teleop_node,),
        daemon=True,
    )
    thread.start()

    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
