import math
import time as t

import rclpy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator

class LidarRecovery:
    """
    Recovery lidar basée sur les angles du LaserScan.

    Elle publie des commandes Ackermann sur un topic de recovery
    et publie un Bool enable pour dire explicitement au mux
    quand elle prend ou rend le contrôle.

    Convention avec le mux:
      - recovery_enable = True  : la recovery LiDAR contrôle la voiture
      - recovery_enable = False : le mux peut repasser à une priorité plus basse

    Les commandes Ackermann nulles restent autorisées pendant enable=True:
    elles signifient alors "arrêt contrôlé par la recovery", pas "recovery inactive".
    """

    def __init__(self, navigator: BasicNavigator):
        self.node = navigator
        self.logger = navigator.get_logger()

        self.node.declare_parameter("recovery_scan_topic", "/scan")
        self.node.declare_parameter("recovery_cmd_topic", "/yellow_car/cmd_ackermann_recovery")
        self.node.declare_parameter("recovery_enable_topic", "/yellow_car/recovery_enable")
        self.node.declare_parameter("recovery_duration", 1.0)
        self.node.declare_parameter("recovery_rate", 10.0)
        self.node.declare_parameter("recovery_forward_speed", 1.0)
        self.node.declare_parameter("recovery_reverse_speed", -0.6)
        self.node.declare_parameter("recovery_forward_clear_distance", 0.35)
        self.node.declare_parameter("recovery_max_steering_angle", 0.28)
        self.node.declare_parameter("recovery_steering_gain", 1.0)
        self.node.declare_parameter("recovery_front_center_index", -1)
        self.node.declare_parameter("recovery_invert_lidar_order", False)
        self.node.declare_parameter("recovery_invert_steering", False)

        # Topic utilisé uniquement pour détecter que Nav2 est actif.
        # La commande réelle de la voiture passe par /cmd_twist dans le mux.
        # Dès que Nav2 republie une commande non nulle ici, la recovery s'arrête.
        self.node.declare_parameter("nav_cmd_topic", "/cmd_vel_nav")
        self.node.declare_parameter("nav_cmd_timeout", 0.30)
        self.node.declare_parameter("nav_cmd_min_linear", 0.01)
        self.node.declare_parameter("nav_cmd_min_angular", 0.01)

        # Convention LaserScan:
        #   angle 0 rad  = avant
        #   angle > 0    = gauche
        #   angle < 0    = droite
        self.node.declare_parameter("recovery_front_half_angle_deg", 15.0)
        self.node.declare_parameter("recovery_side_angle_deg", 105.0)

        self.scan_topic = str(self.node.get_parameter("recovery_scan_topic").value)
        self.cmd_topic = str(self.node.get_parameter("recovery_cmd_topic").value)
        self.enable_topic = str(self.node.get_parameter("recovery_enable_topic").value)
        self.duration = float(self.node.get_parameter("recovery_duration").value)
        self.rate = float(self.node.get_parameter("recovery_rate").value)
        self.forward_speed = float(self.node.get_parameter("recovery_forward_speed").value)
        self.reverse_speed = float(self.node.get_parameter("recovery_reverse_speed").value)
        self.forward_clear_distance = float(
            self.node.get_parameter("recovery_forward_clear_distance").value
        )
        self.max_steering = float(
            self.node.get_parameter("recovery_max_steering_angle").value
        )
        self.steering_gain = float(self.node.get_parameter("recovery_steering_gain").value)
        self.front_center_index = int(
            self.node.get_parameter("recovery_front_center_index").value
        )
        self.invert_lidar_order = bool(
            self.node.get_parameter("recovery_invert_lidar_order").value
        )
        self.invert_steering = bool(
            self.node.get_parameter("recovery_invert_steering").value
        )
        self.nav_cmd_topic = str(self.node.get_parameter("nav_cmd_topic").value)
        self.nav_cmd_timeout = float(self.node.get_parameter("nav_cmd_timeout").value)
        self.nav_cmd_min_linear = float(self.node.get_parameter("nav_cmd_min_linear").value)
        self.nav_cmd_min_angular = float(self.node.get_parameter("nav_cmd_min_angular").value)
        self.last_nav_cmd_time = None
        self.front_half_angle = math.radians(
            float(self.node.get_parameter("recovery_front_half_angle_deg").value)
        )
        self.side_angle = math.radians(
            float(self.node.get_parameter("recovery_side_angle_deg").value)
        )

        self.scan_points = None
        self.cmd_pub = self.node.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.enable_pub = self.node.create_publisher(Bool, self.enable_topic, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        self.nav_cmd_sub = self.node.create_subscription(
            Twist,
            self.nav_cmd_topic,
            self.nav_cmd_callback,
            10,
        )

        self.logger.info(
            f"Lidar recovery ready: scan={self.scan_topic}, cmd={self.cmd_topic}, enable={self.enable_topic}, "
            f"front=±{math.degrees(self.front_half_angle):.1f}deg, "
            f"sides=±{math.degrees(self.side_angle):.1f}deg"
        )

    def reset_nav_cmd_watch(self) -> None:
        self.last_nav_cmd_time = None

    def nav_cmd_callback(self, msg: Twist) -> None:
        moving_cmd = (
            abs(float(msg.linear.x)) > self.nav_cmd_min_linear
            or abs(float(msg.angular.z)) > self.nav_cmd_min_angular
        )
        if moving_cmd:
            self.last_nav_cmd_time = t.monotonic()

    def nav2_has_fresh_cmd(self) -> bool:
        if self.last_nav_cmd_time is None:
            return False
        return (t.monotonic() - self.last_nav_cmd_time) <= self.nav_cmd_timeout

    def normalize_angle(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def clean_range(self, r: float) -> float:
        if math.isnan(r) or math.isinf(r):
            return 6.0
        if r <= 0.0:
            return 0.0
        return min(float(r), 6.0)

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = [self.clean_range(r) for r in msg.ranges]

        if self.invert_lidar_order:
            ranges = list(reversed(ranges))

        n = len(ranges)
        if n == 0:
            self.scan_points = None
            return

        front_angle_offset = 0.0
        if self.front_center_index >= 0:
            center = max(0, min(n - 1, self.front_center_index))
            front_angle_offset = msg.angle_min + center * msg.angle_increment

        points = []
        for i, r in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment - front_angle_offset
            points.append((self.normalize_angle(angle), r))

        self.scan_points = points

    def publish_enable(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub.publish(msg)

    def publish_cmd(self, speed: float, steering: float) -> None:
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        self.publish_cmd(0.0, 0.0)
        self.publish_enable(False)

    def zone_mean(self, values):
        valid = [v for v in values if v > 0.0]
        if not valid:
            return 6.0
        return sum(valid) / len(valid)

    def zone_min(self, values):
        valid = [v for v in values if v > 0.0]
        if not valid:
            return 6.0
        return min(valid)

    def extract_zones(self):
        left = []
        front = []
        right = []

        for angle, distance in self.scan_points:
            if abs(angle) <= self.front_half_angle:
                front.append(distance)
            elif self.front_half_angle < angle <= self.side_angle:
                left.append(distance)
            elif -self.side_angle <= angle < -self.front_half_angle:
                right.append(distance)

        return left, front, right

    def compute_cmd(self):
        if self.scan_points is None or len(self.scan_points) < 9:
            return 0.0, 0.0, 0.0, "NO_SCAN"

        left, front, right = self.extract_zones()

        if not front:
            return 0.0, 0.0, 0.0, "NO_FRONT_ZONE"

        left_m = self.zone_mean(left)
        right_m = self.zone_mean(right)
        front_min = self.zone_min(front)

        # Si la droite est plus libre, right_m > left_m => steering négatif.
        steering = -self.steering_gain * (right_m - left_m) / max(0.20, front_min)
        steering = max(-1.0, min(1.0, steering)) * self.max_steering

        if self.invert_steering:
            steering = -steering

        if front_min >= self.forward_clear_distance:
            # Il y a de la place devant: on avance vers le côté le plus libre.
            return self.forward_speed, steering, front_min, "FORWARD"

        # Devant bloqué: on recule.
        # En marche arrière on inverse le braquage pour faire pivoter l'avant
        # vers le côté qui semblait le plus libre.
        return self.reverse_speed, -steering, front_min, "REVERSE"

    def run(self, stop_on_nav_cmd: bool = True) -> str:
        self.logger.warn("Recovery: lidar forward-or-reverse")
        self.publish_enable(True)

        start_wait = t.monotonic()
        while self.scan_points is None and (t.monotonic() - start_wait) < 1.0:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.scan_points is None:
            self.logger.error("Recovery impossible: aucun LaserScan reçu")
            self.stop()
            return "NO_SCAN"

        dt = 1.0 / max(1.0, self.rate)
        start = t.monotonic()

        while (t.monotonic() - start) < self.duration:
            rclpy.spin_once(self.node, timeout_sec=0.0)

            if stop_on_nav_cmd and self.nav2_has_fresh_cmd():
                self.logger.info("Recovery arrêtée: /cmd_vel_nav actif")
                self.stop()
                return "NAV_CMD_ACTIVE"

            speed, steering, front_min, mode = self.compute_cmd()
            self.publish_cmd(speed, steering)
            self.logger.info(
                f"RECOVERY {mode} front={front_min:.2f} "
                f"speed={speed:.2f} steering={steering:.2f}",
                throttle_duration_sec=0.5,
            )
            t.sleep(dt)

        self.stop()
        return "TIMEOUT"