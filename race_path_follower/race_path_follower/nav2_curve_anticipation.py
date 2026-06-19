#!/usr/bin/env python3

"""
Nav2CurveAnticipation

Classe Python importable dans improved_race_node.py.
Elle adapte les paramètres du controller Nav2 avant les virages en analysant /plan.

Elle n'est PAS un node séparé : elle utilise le BasicNavigator comme node ROS.

Topics lus :
  /plan : nav_msgs/msg/Path
  /odom : nav_msgs/msg/Odometry

Paramètres Nav2 modifiés dans /controller_server :
  FollowPath.desired_linear_vel
  FollowPath.max_lookahead_dist
  FollowPath.regulated_linear_scaling_min_speed, optionnel
"""

import math
import time as t
from typing import List, Optional, Tuple

from nav_msgs.msg import Path, Odometry

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


Point2D = Tuple[float, float]


class Nav2CurveAnticipation:
    def __init__(
        self,
        node,
        logger,
        plan_topic='/plan',
        odom_topic='/odom',
        controller_server='/controller_server',
        speed_param_name='FollowPath.desired_linear_vel',
        lookahead_param_name='FollowPath.max_lookahead_dist',
        regulated_min_speed_param_name='FollowPath.regulated_linear_scaling_min_speed',
        change_regulated_min_speed=True,
        straight_speed=2.5,
        normal_turn_speed=1.6,
        sharp_turn_speed=1.1,
        straight_max_lookahead_dist=1.2,
        normal_turn_max_lookahead_dist=0.8,
        sharp_turn_max_lookahead_dist=0.65,
        # Ancien param conservé pour compatibilité.
        # Si path_end_distance vaut None, on utilise path_check_distance.
        path_check_distance=2.5,
        # Nouvelle fenêtre d'analyse: on ignore le tout début du plan
        # et on analyse seulement [path_start_distance, path_end_distance].
        path_start_distance=0.5,
        path_end_distance=None,
        path_nearest_search_max_points=80,
        turn_heading_change_deg=18.0,
        sharp_turn_heading_change_deg=35.0,
        min_param_update_period=0.20,
        min_mode_duration=0.35,
        log_period=0.50,
    ):
        self.node = node
        self.logger = logger

        self.plan_topic = plan_topic
        self.odom_topic = odom_topic
        self.controller_server = controller_server

        self.speed_param_name = speed_param_name
        self.lookahead_param_name = lookahead_param_name
        self.regulated_min_speed_param_name = regulated_min_speed_param_name
        self.change_regulated_min_speed = bool(change_regulated_min_speed)

        self.straight_speed = float(straight_speed)
        self.normal_turn_speed = float(normal_turn_speed)
        self.sharp_turn_speed = float(sharp_turn_speed)

        self.straight_max_lookahead_dist = float(straight_max_lookahead_dist)
        self.normal_turn_max_lookahead_dist = float(normal_turn_max_lookahead_dist)
        self.sharp_turn_max_lookahead_dist = float(sharp_turn_max_lookahead_dist)

        self.path_start_distance = max(0.0, float(path_start_distance))
        if path_end_distance is None:
            self.path_end_distance = max(self.path_start_distance + 0.05, float(path_check_distance))
        else:
            self.path_end_distance = max(self.path_start_distance + 0.05, float(path_end_distance))

        # Conservé comme alias pour les logs / compatibilité interne.
        self.path_check_distance = self.path_end_distance
        self.path_nearest_search_max_points = int(path_nearest_search_max_points)
        self.turn_heading_change = math.radians(float(turn_heading_change_deg))
        self.sharp_turn_heading_change = math.radians(float(sharp_turn_heading_change_deg))

        self.min_param_update_period = float(min_param_update_period)
        self.min_mode_duration = float(min_mode_duration)
        self.log_period = float(log_period)

        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None

        self.current_mode = 'UNKNOWN'
        self.last_mode_change_time = 0.0

        self.current_speed_sent: Optional[float] = None
        self.current_lookahead_sent: Optional[float] = None
        self.current_regulated_min_speed_sent: Optional[float] = None

        self.last_param_update_time = 0.0
        self.last_log_time = 0.0

        self.path_sub = self.node.create_subscription(
            Path,
            self.plan_topic,
            self.path_callback,
            10,
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.param_client = self.node.create_client(
            SetParameters,
            self.controller_server + '/set_parameters',
        )

        self.logger.info(
            'Nav2CurveAnticipation ready: '
            f'plan={self.plan_topic}, odom={self.odom_topic}, '
            f'server={self.controller_server}'
        )
        self.logger.info(
            'Curve modes: '
            f'STRAIGHT speed={self.straight_speed:.2f}, lookahead={self.straight_max_lookahead_dist:.2f} | '
            f'TURN speed={self.normal_turn_speed:.2f}, lookahead={self.normal_turn_max_lookahead_dist:.2f} | '
            f'SHARP speed={self.sharp_turn_speed:.2f}, lookahead={self.sharp_turn_max_lookahead_dist:.2f}'
        )
        self.logger.info(
            'Curve path window: '
            f'start={self.path_start_distance:.2f}m, end={self.path_end_distance:.2f}m, '
            f'turn_threshold={math.degrees(self.turn_heading_change):.1f}deg, '
            f'sharp_threshold={math.degrees(self.sharp_turn_heading_change):.1f}deg'
        )

    def path_callback(self, msg: Path) -> None:
        self.latest_path = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    @staticmethod
    def angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def dist(a: Point2D, b: Point2D) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def heading(a: Point2D, b: Point2D) -> Optional[float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if math.hypot(dx, dy) < 1e-4:
            return None
        return math.atan2(dy, dx)

    def get_robot_xy(self) -> Optional[Point2D]:
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        return float(p.x), float(p.y)

    def get_path_points(self) -> List[Point2D]:
        if self.latest_path is None:
            return []
        return [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in self.latest_path.poses
        ]

    def find_nearest_path_index(self, points: List[Point2D], robot_xy: Point2D) -> int:
        if not points:
            return 0

        # Le plan Nav2 est souvent déjà pruned, donc le robot est proche du début.
        # On limite la recherche pour éviter de matcher une autre portion du circuit.
        max_i = min(len(points), max(1, self.path_nearest_search_max_points))

        best_i = 0
        best_d = float('inf')
        for i in range(max_i):
            d = self.dist(points[i], robot_xy)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def collect_future_points_between(
        self,
        points: List[Point2D],
        start_i: int,
        start_distance_m: float,
        end_distance_m: float,
    ) -> List[Point2D]:
        """
        Retourne les points du /plan compris entre deux distances devant le robot.

        Exemple avec start_distance_m=0.5 et end_distance_m=3.0:
          - on ignore les 0.5 premiers metres du plan proche du robot ;
          - on analyse seulement la portion entre 0.5 m et 3.0 m.

        Cela évite qu'une petite correction locale juste devant la voiture soit
        confondue avec un vrai virage de la piste.
        """
        if start_i >= len(points):
            return []

        start_distance_m = max(0.0, float(start_distance_m))
        end_distance_m = max(start_distance_m + 0.05, float(end_distance_m))

        future: List[Point2D] = []
        acc = 0.0
        prev = points[start_i]

        # Si start_distance_m vaut 0, on garde le point le plus proche.
        if start_distance_m <= 1e-6:
            future.append(prev)

        for i in range(start_i + 1, len(points)):
            p = points[i]
            seg_len = self.dist(prev, p)
            next_acc = acc + seg_len

            # On ajoute le point dès que le segment atteint la fenêtre d'analyse.
            # Approximation suffisante pour un Path Nav2 dense.
            if next_acc >= start_distance_m and acc <= end_distance_m:
                if not future:
                    # Ajoute aussi le point précédent pour garder un premier segment.
                    future.append(prev)
                future.append(p)

            acc = next_acc
            prev = p

            if acc >= end_distance_m:
                break

        return future

    # Ancien nom conservé pour compatibilité éventuelle.
    def collect_future_points(self, points: List[Point2D], start_i: int, distance_m: float) -> List[Point2D]:
        return self.collect_future_points_between(points, start_i, 0.0, distance_m)

    def compute_path_heading_metrics(self) -> Optional[Tuple[float, float]]:
        """
        Retourne:
          max_heading_change_rad, distance_to_first_turn_m
        """
        robot_xy = self.get_robot_xy()
        points = self.get_path_points()

        if robot_xy is None or len(points) < 4:
            return None

        nearest_i = self.find_nearest_path_index(points, robot_xy)
        future = self.collect_future_points_between(
            points,
            nearest_i,
            self.path_start_distance,
            self.path_end_distance,
        )

        if len(future) < 4:
            return None

        headings: List[float] = []
        seg_lengths: List[float] = []

        for i in range(1, len(future)):
            h = self.heading(future[i - 1], future[i])
            if h is None:
                continue
            headings.append(h)
            seg_lengths.append(self.dist(future[i - 1], future[i]))

        if len(headings) < 2:
            return None

        ref = headings[0]
        max_change = 0.0
        distance_to_first_turn = self.path_end_distance
        acc_dist = 0.0
        found_first_turn = False

        for i, h in enumerate(headings[1:], start=1):
            if i - 1 < len(seg_lengths):
                acc_dist += seg_lengths[i - 1]

            change = abs(self.angle_diff(h, ref))
            max_change = max(max_change, change)

            if (not found_first_turn) and change >= self.turn_heading_change:
                distance_to_first_turn = acc_dist
                found_first_turn = True

        return max_change, distance_to_first_turn

    def select_target_mode(self) -> Tuple[str, Optional[float], Optional[float]]:
        metrics = self.compute_path_heading_metrics()

        if metrics is None:
            # Si on ne sait pas, mode prudent.
            return 'TURN', None, None

        heading_change, turn_distance = metrics

        if heading_change >= self.sharp_turn_heading_change:
            return 'SHARP_TURN', heading_change, turn_distance

        if heading_change >= self.turn_heading_change:
            return 'TURN', heading_change, turn_distance

        return 'STRAIGHT', heading_change, turn_distance

    def apply_mode_hysteresis(self, target_mode: str) -> str:
        now = t.monotonic()

        if self.current_mode == 'UNKNOWN':
            self.current_mode = target_mode
            self.last_mode_change_time = now
            return self.current_mode

        if target_mode == self.current_mode:
            return self.current_mode

        # Passage immédiat vers plus prudent.
        priority = {
            'STRAIGHT': 0,
            'TURN': 1,
            'SHARP_TURN': 2,
        }
        current_p = priority.get(self.current_mode, 1)
        target_p = priority.get(target_mode, 1)

        if target_p > current_p:
            self.current_mode = target_mode
            self.last_mode_change_time = now
            return self.current_mode

        # Retour vers plus rapide seulement après une durée minimale.
        if (now - self.last_mode_change_time) >= self.min_mode_duration:
            self.current_mode = target_mode
            self.last_mode_change_time = now

        return self.current_mode

    def mode_to_values(self, mode: str) -> Tuple[float, float, float]:
        if mode == 'STRAIGHT':
            return (
                self.straight_speed,
                self.straight_max_lookahead_dist,
                self.normal_turn_speed,
            )

        if mode == 'SHARP_TURN':
            return (
                self.sharp_turn_speed,
                self.sharp_turn_max_lookahead_dist,
                self.sharp_turn_speed,
            )

        return (
            self.normal_turn_speed,
            self.normal_turn_max_lookahead_dist,
            self.normal_turn_speed,
        )

    def make_double_param(self, name: str, value: float) -> Parameter:
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(value)
        return param

    def set_controller_params(self, speed: float, lookahead: float, regulated_min_speed: float) -> None:
        now = t.monotonic()

        if (now - self.last_param_update_time) < self.min_param_update_period:
            return

        changed = False

        if self.current_speed_sent is None or abs(self.current_speed_sent - speed) > 1e-4:
            changed = True

        if self.current_lookahead_sent is None or abs(self.current_lookahead_sent - lookahead) > 1e-4:
            changed = True

        if self.change_regulated_min_speed:
            if (
                self.current_regulated_min_speed_sent is None
                or abs(self.current_regulated_min_speed_sent - regulated_min_speed) > 1e-4
            ):
                changed = True

        if not changed:
            return

        if not self.param_client.service_is_ready():
            self.logger.warn(
                self.controller_server + '/set_parameters pas encore prêt',
                throttle_duration_sec=1.0,
            )
            return

        params = [
            self.make_double_param(self.speed_param_name, speed),
            self.make_double_param(self.lookahead_param_name, lookahead),
        ]

        if self.change_regulated_min_speed:
            params.append(
                self.make_double_param(self.regulated_min_speed_param_name, regulated_min_speed)
            )

        req = SetParameters.Request()
        req.parameters = params
        self.param_client.call_async(req)

        self.current_speed_sent = speed
        self.current_lookahead_sent = lookahead
        if self.change_regulated_min_speed:
            self.current_regulated_min_speed_sent = regulated_min_speed

        self.last_param_update_time = now

    def update(self) -> None:
        target_mode, heading_change, turn_distance = self.select_target_mode()
        mode = self.apply_mode_hysteresis(target_mode)
        speed, lookahead, regulated_min_speed = self.mode_to_values(mode)

        self.set_controller_params(speed, lookahead, regulated_min_speed)

        now = t.monotonic()
        if (now - self.last_log_time) >= self.log_period:
            self.last_log_time = now
            if heading_change is None:
                self.logger.info(
                    f'Curve anticipation: mode={mode}, no valid path metrics, '
                    f'speed={speed:.2f}, lookahead={lookahead:.2f}'
                )
            else:
                self.logger.info(
                    f'Curve anticipation: target={target_mode}, mode={mode}, '
                    f'heading_change={math.degrees(heading_change):.1f}deg, '
                    f'turn_dist={turn_distance:.2f}m, '
                    f'window={self.path_start_distance:.2f}-{self.path_end_distance:.2f}m, '
                    f'speed={speed:.2f}, lookahead={lookahead:.2f}'
                )
