#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive


class DialNavBlue(Node):
    """
    Navigation lidar par secteurs (dials) pour blue_car.

    Entrée:
      /blue_car/scan

    Sortie:
      /blue_car/cmd_ackermann

    Fonctionnalités:
      - recentrage du lidar avec front_center_index
      - inversion optionnelle du lidar
      - inversion optionnelle de la direction
      - navigation 3Dials ou NDials
      - marche arrière de dégagement si obstacle trop proche devant
      - aucune vérification arrière: lidar avant 200 degrés uniquement
    """

    def __init__(self):
        super().__init__("dial_nav_blue")

        # Topics
        self.declare_parameter("scan_topic", "/blue_car/scan")
        self.declare_parameter("cmd_topic", "/blue_car/cmd_ackermann")

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        # Calibration lidar
        self.declare_parameter("front_center_index", 180)
        self.declare_parameter("invert_lidar_order", False)
        self.declare_parameter("invert_steering", False)

        self.front_center_index = int(self.get_parameter("front_center_index").value)
        self.invert_lidar_order = bool(self.get_parameter("invert_lidar_order").value)
        self.invert_steering = bool(self.get_parameter("invert_steering").value)

        # Navigation
        self.declare_parameter("mode", "3Dials")  # "3Dials" ou "NDials"
        self.declare_parameter("n_dials", 7)
        self.declare_parameter("spaced_dials", True)
        self.declare_parameter("front_dial_ratio", 0.2)
        self.declare_parameter("feature", "median")  # mean, median, min, max

        self.mode = self.get_parameter("mode").value
        self.n_dials = int(self.get_parameter("n_dials").value)
        self.spaced_dials = bool(self.get_parameter("spaced_dials").value)
        self.front_dial_ratio = float(self.get_parameter("front_dial_ratio").value)
        self.feature = self.get_parameter("feature").value

        # Gains
        self.declare_parameter("Kspeed", 0.8)
        self.declare_parameter("Kdir", 1.2)
        self.declare_parameter("Karg", 0.2)

        self.Kspeed = float(self.get_parameter("Kspeed").value)
        self.Kdir = float(self.get_parameter("Kdir").value)
        self.Karg = float(self.get_parameter("Karg").value)

        # Limites Ackermann
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("min_speed", 0.20)
        self.declare_parameter("max_steering_angle", 0.28)

        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)

        # Filtrage
        self.declare_parameter("alpha_speed", 1.0)#0.4)
        self.declare_parameter("alpha_steering", 1.0)#0.4)

        self.alpha_speed = float(self.get_parameter("alpha_speed").value)
        self.alpha_steering = float(self.get_parameter("alpha_steering").value)

        self.last_speed = 0.0
        self.last_steering = 0.0

        # Marche arrière / dégagement
        self.declare_parameter("enable_reverse", True)
        self.declare_parameter("emergency_distance", 0.35)
        self.declare_parameter("reverse_release_distance", 0.55)
        self.declare_parameter("reverse_speed", -0.35)
        self.declare_parameter("reverse_steps", 25)
        self.declare_parameter("min_reverse_steps", 8)
        self.declare_parameter("reverse_stop_steps", 3)
        self.declare_parameter("reverse_cooldown_steps", 8)
        self.declare_parameter("reverse_steering_mode", "auto")  # auto, last_opposite, straight
        self.declare_parameter("reverse_steering_gain", 0.9)
        self.declare_parameter("front_emergency_ratio", 0.08)

        self.enable_reverse = bool(self.get_parameter("enable_reverse").value)
        self.emergency_distance = float(self.get_parameter("emergency_distance").value)
        self.reverse_release_distance = float(
            self.get_parameter("reverse_release_distance").value
        )
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)
        self.reverse_steps = int(self.get_parameter("reverse_steps").value)
        self.min_reverse_steps = int(self.get_parameter("min_reverse_steps").value)
        self.reverse_stop_steps = int(self.get_parameter("reverse_stop_steps").value)
        self.reverse_cooldown_steps = int(self.get_parameter("reverse_cooldown_steps").value)
        self.reverse_steering_mode = self.get_parameter("reverse_steering_mode").value
        self.reverse_steering_gain = float(
            self.get_parameter("reverse_steering_gain").value
        )
        self.front_emergency_ratio = float(self.get_parameter("front_emergency_ratio").value)

        self.reverse_remaining = 0
        self.reverse_done_steps = 0
        self.reverse_steering = 0.0
        self.reverse_stop_remaining = 0
        self.reverse_cooldown_remaining = 0

        self.cmd_pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        self.get_logger().info("Dial navigation BLUE démarrée")
        self.get_logger().info(f"scan_topic={self.scan_topic}")
        self.get_logger().info(f"cmd_topic={self.cmd_topic}")
        self.get_logger().info(
            f"front_center_index={self.front_center_index}, "
            f"invert_lidar_order={self.invert_lidar_order}, "
            f"invert_steering={self.invert_steering}"
        )

    def clean_ranges(self, ranges):
        clean = []
        for r in ranges:
            if math.isnan(r) or math.isinf(r):
                clean.append(6.0)
            elif r <= 0.0:
                clean.append(0.0)
            else:
                clean.append(min(float(r), 6.0))
        return clean

    def recenter_lidar(self, ranges):
        """
        Réorganise le lidar pour que l'avant soit au centre du tableau.
        Après cette fonction, ranges[n//2] correspond à l'avant.
        """
        n = len(ranges)
        if n == 0:
            return ranges

        if self.invert_lidar_order:
            ranges = list(reversed(ranges))

        center = max(0, min(n - 1, self.front_center_index))
        target_center = n // 2
        shift = target_center - center

        if shift == 0:
            return ranges
        return ranges[-shift:] + ranges[:-shift]

    def get_feature_func(self):
        if self.feature == "mean":
            return np.mean
        if self.feature == "min":
            return np.min
        if self.feature == "max":
            return np.max
        return np.median

    def get_dials_ranges(self, ranges, n_dials=3, proportion=None):
        if len(ranges) < n_dials:
            return None

        if proportion is None:
            proportion = [1] * n_dials

        if len(proportion) != n_dials:
            proportion = [1] * n_dials

        proportion = len(ranges) * (np.array(proportion, dtype=float) / sum(proportion))
        proportion = np.round(proportion).astype(int)

        # Corrige l'arrondi pour tomber exactement sur len(ranges)
        diff = len(ranges) - int(np.sum(proportion))
        proportion[-1] += diff

        indices = np.cumsum(proportion)[:-1]
        indices = np.insert(indices, 0, 0)
        indices = np.append(indices, len(ranges))

        dials = []
        for i in range(n_dials):
            dials.append(np.array(ranges[indices[i]:indices[i + 1]], dtype=float))
        return dials

    def crop(self, value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def safe_min(self, values):
        valid = [r for r in values if r > 0.0]
        return min(valid) if len(valid) > 0 else 999.0

    def get_front_distances(self, ranges):
        """
        À appeler après recenter_lidar().

        Le lidar ne voit que l'avant (~200 degrés), donc on ne tente pas
        d'estimer une distance arrière. On utilise seulement la zone avant
        pour déclencher la marche arrière et choisir le braquage.
        """
        n = len(ranges)
        if n == 0:
            return {
                "front": 999.0,
                "front_left": 999.0,
                "front_right": 999.0,
            }

        center = n // 2
        width = max(3, int(self.front_emergency_ratio * n))

        front_left_zone = ranges[max(0, center - width):center]
        front_right_zone = ranges[center + 1:min(n, center + width + 1)]
        front_zone = front_left_zone + [ranges[center]] + front_right_zone

        return {
            "front": self.safe_min(front_zone),
            "front_left": self.safe_min(front_left_zone),
            "front_right": self.safe_min(front_right_zone),
        }

    def forward_3_dials(self, ranges):
        if self.spaced_dials:
            dials = self.get_dials_ranges(
                ranges,
                n_dials=5,
                proportion=[70, 10, 20, 10, 70],
            )
            if dials is None:
                return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            range_left = dials[0]
            range_center = dials[2]
            range_right = dials[4]
        else:
            dials_2 = self.get_dials_ranges(ranges, n_dials=2)
            dials_3 = self.get_dials_ranges(
                ranges,
                n_dials=3,
                proportion=[
                    1.0 - self.front_dial_ratio,
                    2.0 * self.front_dial_ratio,
                    1.0 - self.front_dial_ratio,
                ],
            )
            if dials_2 is None or dials_3 is None:
                return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            range_left, range_right = dials_2
            range_center = dials_3[1]

        feature = self.get_feature_func()
        dist_left = float(feature(range_left))
        dist_center = float(feature(range_center))
        dist_right = float(feature(range_right))
        dist_center = max(dist_center, 0.05)

        mid_i = len(ranges) // 2
        arg = -float(np.argmax(ranges) - mid_i) / float(mid_i)

        speed_cmd = self.Kspeed * dist_center
        dir_cmd = -self.Kdir * (dist_right - dist_left) / dist_center + self.Karg * arg

        return speed_cmd, dir_cmd, dist_left, dist_center, dist_right, arg

    def forward_n_dials(self, ranges):
        feature = self.get_feature_func()

        if self.spaced_dials:
            dials_5 = self.get_dials_ranges(
                ranges,
                n_dials=5,
                proportion=[70, 10, 20, 10, 70],
            )
            if dials_5 is None:
                return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            range_left = dials_5[0]
            range_center = dials_5[2]
            range_right = dials_5[4]
        else:
            dials_2 = self.get_dials_ranges(ranges, n_dials=2)
            dials_3 = self.get_dials_ranges(
                ranges,
                n_dials=3,
                proportion=[
                    1.0 - self.front_dial_ratio,
                    2.0 * self.front_dial_ratio,
                    1.0 - self.front_dial_ratio,
                ],
            )
            if dials_2 is None or dials_3 is None:
                return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            range_left, range_right = dials_2
            range_center = dials_3[1]

        dial_ranges = self.get_dials_ranges(ranges, n_dials=self.n_dials)
        if dial_ranges is None:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        dial_values = np.array([feature(dial) for dial in dial_ranges], dtype=float)

        dist_left = float(feature(range_left))
        dist_center = float(feature(range_center))
        dist_right = float(feature(range_right))
        dist_center = max(dist_center, 0.05)

        mid_i = len(dial_values) // 2
        arg = -float(np.argmax(dial_values) - mid_i) / float(mid_i)

        speed_cmd = self.Kspeed * dist_center
        dir_cmd = -self.Kdir * (dist_right - dist_left) / dist_center + self.Karg * arg

        return speed_cmd, dir_cmd, dist_left, dist_center, dist_right, arg

    def publish_raw_cmd(self, speed, steering):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering)
        self.cmd_pub.publish(msg)

    def publish_cmd(self, speed_cmd, dir_cmd):
        dir_cmd = self.crop(dir_cmd, -1.0, 1.0)

        if self.invert_steering:
            dir_cmd = -dir_cmd

        speed = self.crop(speed_cmd, self.min_speed, self.max_speed)
        steering = dir_cmd * self.max_steering_angle

        speed = (1.0 - self.alpha_speed) * self.last_speed + self.alpha_speed * speed
        steering = (
            (1.0 - self.alpha_steering) * self.last_steering
            + self.alpha_steering * steering
        )

        self.last_speed = speed
        self.last_steering = steering

        self.publish_raw_cmd(speed, steering)

    def stop(self):
        self.publish_raw_cmd(0.0, 0.0)

    def choose_reverse_steering(self, distances):
        """
        Choisit un braquage de marche arrière au début du dégagement.

        Comme on n'a pas de capteur arrière, le mode auto utilise seulement
        ce que le lidar voit devant: si l'avant gauche est plus libre, on
        braque dans un sens; si l'avant droit est plus libre, dans l'autre.
        """
        mode = str(self.reverse_steering_mode)

        if mode == "straight":
            steering = 0.0
        elif mode == "last_opposite":
            steering = -self.last_steering
        else:
            # Choix basé uniquement sur le scan avant.
            direction = 1.0 if distances["front_right"] >= distances["front_left"] else -1.0
            steering = direction * self.reverse_steering_gain * self.max_steering_angle

        steering = self.crop(
            steering,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        if self.invert_steering:
            steering = -steering

        return steering

    def reverse_step(self):
        """Publie une commande de recul avec le braquage mémorisé."""
        self.last_speed = self.reverse_speed
        self.last_steering = self.reverse_steering
        self.publish_raw_cmd(self.reverse_speed, self.reverse_steering)

    def start_reverse(self, distances):
        self.reverse_remaining = self.reverse_steps
        self.reverse_done_steps = 0
        self.reverse_steering = self.choose_reverse_steering(distances)
        self.get_logger().warn(
            "Obstacle avant "
            f"{distances['front']:.2f} m -> marche arrière "
            f"steer={self.reverse_steering:.2f} "
            f"FL={distances['front_left']:.2f} FR={distances['front_right']:.2f}"
        )
        self.reverse_step()

    def finish_reverse(self):
        self.reverse_remaining = 0
        self.reverse_done_steps = 0
        self.reverse_steering = 0.0
        self.reverse_stop_remaining = self.reverse_stop_steps
        self.reverse_cooldown_remaining = self.reverse_cooldown_steps
        self.stop()

    def scan_callback(self, msg):
        ranges = self.clean_ranges(list(msg.ranges))

        if len(ranges) == 0:
            self.stop()
            return

        ranges = self.recenter_lidar(ranges)
        distances = self.get_front_distances(ranges)
        min_front = distances["front"]

        # Petite pause après un recul: évite de repartir immédiatement dans
        # l'obstacle avec une commande filtrée encore négative/forte.
        if self.enable_reverse and self.reverse_stop_remaining > 0:
            self.reverse_stop_remaining -= 1
            self.last_speed = 0.0
            self.last_steering = 0.0
            self.stop()
            return

        if self.reverse_cooldown_remaining > 0:
            self.reverse_cooldown_remaining -= 1

        # Marche arrière en cours
        if self.enable_reverse and self.reverse_remaining > 0:
            self.reverse_remaining -= 1
            self.reverse_done_steps += 1

            reverse_can_stop = self.reverse_done_steps >= self.min_reverse_steps
            front_is_clear = min_front > self.reverse_release_distance
            if reverse_can_stop and front_is_clear:
                self.get_logger().warn(
                    f"Fin recul: avant dégagé à {min_front:.2f} m "
                    f"après {self.reverse_done_steps} steps"
                )
                self.finish_reverse()
                return

            self.get_logger().warn(
                f"REVERSE: front={min_front:.2f} "
                f"steps_left={self.reverse_remaining} steer={self.reverse_steering:.2f}"
            )
            self.reverse_step()
            return

        # Déclenchement marche arrière
        can_start_reverse = self.reverse_cooldown_remaining == 0
        if self.enable_reverse and can_start_reverse and min_front < self.emergency_distance:
            self.start_reverse(distances)
            return

        if self.enable_reverse and min_front < self.emergency_distance:
            self.stop()
            return

        if self.mode == "NDials":
            speed_cmd, dir_cmd, dl, dc, dr, arg = self.forward_n_dials(ranges)
        else:
            speed_cmd, dir_cmd, dl, dc, dr, arg = self.forward_3_dials(ranges)

        self.get_logger().info(
            f"mode={self.mode} "
            f"L={dl:.2f} C={dc:.2f} R={dr:.2f} "
            f"front={min_front:.2f} "
            f"arg={arg:.2f} dir={dir_cmd:.2f} speed={speed_cmd:.2f}"
        )

        self.publish_cmd(speed_cmd, dir_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DialNavBlue()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
