import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Noeudconduite(Node):
    # Gain appliqué à l'écart de distance gauche/droite pour produire
    # une consigne de braquage en radians.
    _STEERING_GAIN_RAD_PER_M = math.radians(110.0)
    # Saturation de la consigne pour rester dans une plage réaliste.
    _MAX_STEERING_ANGLE_RAD = math.radians(18.0)

    def __init__(self):
        super().__init__('car_drive_node')
        self.declare_parameter('lidar_topic', '/yellow_car/scan')
        self.declare_parameter('ackermann_topic', '/yellow_car/cmd_ackermann')
        self._lidar_topic = self.get_parameter(
            'lidar_topic').get_parameter_value().string_value
        self._ackermann_topic = self.get_parameter(
            'ackermann_topic').get_parameter_value().string_value

        # ROS interface
        self.__ackermann_publisher = self.create_publisher(
            AckermannDrive,
            self._ackermann_topic,
            1,
        )
        self.create_subscription(
            LaserScan,
            self._lidar_topic,
            self.__on_lidar_acquisition,
            1,
        )
        self.get_logger().info(
            "Noeud créé, souscription Lidar sur '%s'" % self._lidar_topic
        )
        self.get_logger().info(
            "Publication Ackermann sur '%s'" % self._ackermann_topic
        )

    def __angle_to_index(self, message, angle_rad):
        # Convertit un angle en indice dans `ranges`, y compris si
        # le scan est publié avec un incrément angulaire négatif.
        if not message.ranges or message.angle_increment == 0.0:
            return None

        raw_index = round(
            (angle_rad - message.angle_min) / message.angle_increment
        )
        return min(max(raw_index, 0), len(message.ranges) - 1)

    def __range_at_angle(self, message, angle_rad):
        index = self.__angle_to_index(message, angle_rad)
        if index is None:
            return None

        # Ignore les mesures invalides avant le calcul de la commande.
        lidar_range = message.ranges[index]
        if not math.isfinite(lidar_range):
            return None
        return lidar_range

    def __on_lidar_acquisition(self, message):
        # On compare deux directions symétriques pour orienter la voiture
        # vers le côté où l'espace libre semble plus grand.
        left_range = self.__range_at_angle(message, math.radians(60.0))
        right_range = self.__range_at_angle(message, math.radians(-60.0))

        # Message de commande du véhicule
        command_message = AckermannDrive()

        if left_range is None or right_range is None:
            self.get_logger().warning(
                'Mesures lidar invalides, commande neutre publiée.'
            )
            self.__ackermann_publisher.publish(command_message)
            return

        # self.get_logger().info(
        #     f'60 {left_range:.2f} et -60 {right_range:.2f}'
        # )

        # Vitesse linéaire constante, choisie empiriquement
        command_message.speed = 0.85
        # L'écart de distance est converti directement en braquage.
        command_message.steering_angle = (
            self._STEERING_GAIN_RAD_PER_M * (left_range - right_range)
        )
        # On borne le braquage avant publication.
        if command_message.steering_angle > self._MAX_STEERING_ANGLE_RAD:
            command_message.steering_angle = self._MAX_STEERING_ANGLE_RAD
        if command_message.steering_angle < -self._MAX_STEERING_ANGLE_RAD:
            command_message.steering_angle = -self._MAX_STEERING_ANGLE_RAD

        # Publication
        self.__ackermann_publisher.publish(command_message)

        # Log
        self.get_logger().info(
            f'v = {command_message.speed:.2f} m/s, '
            f'dir = {command_message.steering_angle:.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    noeud = Noeudconduite()
    rclpy.spin(noeud)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
