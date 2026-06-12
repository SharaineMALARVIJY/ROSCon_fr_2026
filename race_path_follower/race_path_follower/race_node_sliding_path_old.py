import math
import time as t
from typing import List, Optional, Tuple

import yaml

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


Point = Tuple[float, float, float]


class RaceNavigator(BasicNavigator):
    """
    BasicNavigator + lecture de la position actuelle via /amcl_pose.

    On utilise cette position pour savoir quand la voiture est proche du waypoint
    qui sert de point de changement de fenêtre.
    """

    def __init__(self) -> None:
        super().__init__(node_name="race_node")

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10,
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def has_pose(self) -> bool:
        return self.current_x is not None and self.current_y is not None

    def distance_to_point(self, p: Point) -> float:
        if not self.has_pose():
            return float("inf")

        dx = self.current_x - float(p[0])
        dy = self.current_y - float(p[1])
        return math.sqrt(dx * dx + dy * dy)


# -----------------------------------------------------------------------------
# Fonctions utilitaires
# -----------------------------------------------------------------------------


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(navigator: BasicNavigator, p: Point, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(p[0])
    pose.pose.position.y = float(p[1])
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(float(p[2]))
    return pose


def validate_point(logger, name: str, p) -> bool:
    if not isinstance(p, (list, tuple)) or len(p) != 3:
        logger.error(f"{name} must contain exactly 3 values: [x, y, yaw]")
        return False

    try:
        float(p[0])
        float(p[1])
        float(p[2])
    except (TypeError, ValueError):
        logger.error(f"{name} values must be numeric: [x, y, yaw]")
        return False

    return True


def load_track_points(tracks_file: str, active_track: str):
    with open(tracks_file, "r") as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError("tracks.yaml est vide.")

    if "tracks" not in data:
        raise ValueError("tracks.yaml doit contenir une section 'tracks'.")

    if active_track == "":
        if "active_track" not in data:
            raise ValueError(
                "Aucune trajectoire active. "
                "Ajoute 'active_track' dans tracks.yaml ou dans race_params.yaml."
            )
        active_track = data["active_track"]

    if active_track not in data["tracks"]:
        available_tracks = list(data["tracks"].keys())
        raise ValueError(
            f"Trajectoire '{active_track}' introuvable. "
            f"Trajectoires disponibles : {available_tracks}"
        )

    track_data = data["tracks"][active_track]

    if "points" not in track_data:
        raise ValueError(
            f"La trajectoire '{active_track}' ne contient pas de section 'points'."
        )

    raw_points = track_data["points"]

    if not isinstance(raw_points, list):
        raise ValueError(
            f"Les points de la trajectoire '{active_track}' doivent être une liste."
        )

    if len(raw_points) < 3:
        raise ValueError(
            f"La trajectoire '{active_track}' doit contenir au moins 3 points : "
            f"p0 + au moins deux waypoints."
        )

    points: List[Point] = []

    for i, p in enumerate(raw_points):
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(
                f"Point p{i} invalide. Format attendu : [x, y, yaw]."
            )

        points.append((float(p[0]), float(p[1]), float(p[2])))

    return active_track, points


def wait_for_keyboard_start() -> None:
    while True:
        key = input("Tape 'd' puis Entrée pour démarrer : ").strip().lower()
        if key == "d":
            return
        print("Commande invalide. Tape seulement 'd'.")


def append_path(dst: Path, src: Path) -> None:
    """
    Concatène src dans dst en évitant de dupliquer le point de jonction.
    Exemple :
        dst = p0 -> p1
        src = p1 -> p2
        résultat = p0 -> p1 -> p2
    """
    if src is None or not src.poses:
        return

    if not dst.poses:
        dst.header = src.header
        dst.poses = list(src.poses)
        return

    dst.poses.extend(src.poses[1:])


def make_window_path(
    navigator: BasicNavigator,
    p_start: Point,
    p_mid: Point,
    p_end: Point,
    planner_id: str,
    frame_id: str = "map",
    use_start_for_first_segment: bool = True,
) -> Optional[Path]:
    """
    Crée une fenêtre de chemin composée de deux segments :
        p_start -> p_mid -> p_end

    Si use_start_for_first_segment=True :
        segment 1 = p_start -> p_mid

    Si use_start_for_first_segment=False :
        segment 1 = pose actuelle du robot -> p_mid
        Le planner récupère alors le départ via TF.

    Le segment 2 reste toujours :
        p_mid -> p_end

    Puis on concatène les deux chemins.
    """
    logger = navigator.get_logger()

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = navigator.get_clock().now().to_msg()

    if use_start_for_first_segment:
        segment_1 = navigator.getPath(
            start=make_pose(navigator, p_start, frame_id),
            goal=make_pose(navigator, p_mid, frame_id),
            planner_id=planner_id,
            use_start=True,
        )
    else:
        # use_start=False : le planner prend la pose actuelle du robot via TF.
        # On passe quand même un start valide car l'API Python garde l'argument start.
        segment_1 = navigator.getPath(
            start=make_pose(navigator, p_start, frame_id),
            goal=make_pose(navigator, p_mid, frame_id),
            planner_id=planner_id,
            use_start=False,
        )

    if segment_1 is None or not segment_1.poses:
        logger.error("Failed to plan first segment of window")
        return None

    append_path(path, segment_1)

    segment_2 = navigator.getPath(
        start=make_pose(navigator, p_mid, frame_id),
        goal=make_pose(navigator, p_end, frame_id),
        planner_id=planner_id,
        use_start=True,
    )

    if segment_2 is None or not segment_2.poses:
        logger.error("Failed to plan second segment of window")
        return None

    append_path(path, segment_2)

    path.header.stamp = navigator.get_clock().now().to_msg()
    return path


def wait_until_amcl_pose(navigator: RaceNavigator, timeout_sec: float) -> bool:
    """Attend de recevoir au moins une pose AMCL."""
    start_time = t.time()

    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.05)

        if navigator.has_pose():
            return True

        if timeout_sec > 0.0 and (t.time() - start_time) > timeout_sec:
            return False

    return False


def run_recovery(navigator: BasicNavigator, logger) -> None:
    logger.warn("Recovery: backup")

    # disable_collision_checks n'existe pas sur Jazzy
    navigator.backup(
        backup_dist=0.3,
        backup_speed=0.5,
        time_allowance=2,
    )

    while not navigator.isTaskComplete():
        t.sleep(0.025)

    recovery_result = navigator.getResult()
    logger.info(f"Recovery result: {recovery_result}")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------


def main() -> None:
    rclpy.init()

    navigator = RaceNavigator()
    logger = navigator.get_logger()

    # Paramètres généraux
    navigator.declare_parameter("laps", 1)
    navigator.declare_parameter("log_frequency", 10)
    navigator.declare_parameter("planner_id", "GridBased")
    navigator.declare_parameter("controller_id", "FollowPath")
    navigator.declare_parameter("frame_id", "map")

    # Fenêtre glissante
    navigator.declare_parameter("switch_distance", 0.6)
    navigator.declare_parameter("amcl_timeout_sec", 10.0)
    navigator.declare_parameter("use_start_for_first_segment", True)

    # Démarrage clavier
    navigator.declare_parameter("wait_keyboard_start", True)

    # Nouveau chargement des trajectoires
    navigator.declare_parameter("tracks_file", "tracks.yaml")
    navigator.declare_parameter("active_track", "")

    # Lecture des paramètres
    laps = int(navigator.get_parameter("laps").value)
    log_freq = int(navigator.get_parameter("log_frequency").value)
    planner_id = str(navigator.get_parameter("planner_id").value)
    controller_id = str(navigator.get_parameter("controller_id").value)
    frame_id = str(navigator.get_parameter("frame_id").value)

    switch_distance = float(navigator.get_parameter("switch_distance").value)
    amcl_timeout_sec = float(navigator.get_parameter("amcl_timeout_sec").value)
    use_start_for_first_segment = bool(
        navigator.get_parameter("use_start_for_first_segment").value
    )
    wait_keyboard = bool(navigator.get_parameter("wait_keyboard_start").value)

    tracks_file = str(navigator.get_parameter("tracks_file").value)
    active_track = str(navigator.get_parameter("active_track").value)

    # Chargement des points depuis tracks.yaml
    try:
        active_track, all_track_points = load_track_points(tracks_file, active_track)
    except Exception as e:
        logger.error(f"Erreur chargement trajectoire : {e}")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    p0 = all_track_points[0]
    circuit_points = all_track_points[1:]

    # Validation
    if not validate_point(logger, "p0", p0):
        navigator.destroy_node()
        rclpy.shutdown()
        return

    for i, p in enumerate(circuit_points, start=1):
        if not validate_point(logger, f"p{i}", p):
            navigator.destroy_node()
            rclpy.shutdown()
            return

    if len(circuit_points) < 2:
        logger.error(
            "Need at least 2 circuit points after p0. "
            "The track must contain p0, p1 and p2 minimum."
        )
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if laps < 1:
        logger.error("laps must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if log_freq < 1:
        logger.error("log_frequency must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if switch_distance <= 0.0:
        logger.error("switch_distance must be > 0")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    circuit_names = [f"p{i}" for i in range(1, len(circuit_points) + 1)]

    logger.info(f"Tracks file: {tracks_file}")
    logger.info(f"Active track: {active_track}")
    logger.info(f"Initial pose p0: {p0}")
    logger.info(f"Planner: {planner_id}")
    logger.info(f"Controller: {controller_id}")
    logger.info(f"Frame: {frame_id}")
    logger.info(f"Laps: {laps}")
    logger.info(f"Switch distance: {switch_distance:.2f} m")
    logger.info(f"Use start for first segment: {use_start_for_first_segment}")
    logger.info(f"Ordered waypoints: {circuit_names}")

    # Pose initiale pour AMCL / Nav2
    navigator.setInitialPose(make_pose(navigator, p0, frame_id))

    logger.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    logger.info("Nav2 ready")

    logger.info("Waiting for AMCL pose...")
    if not wait_until_amcl_pose(navigator, amcl_timeout_sec):
        logger.error("No AMCL pose received. Check /amcl_pose and localization.")
        navigator.destroy_node()
        rclpy.shutdown()
        return
    logger.info("AMCL pose received")

    if wait_keyboard:
        wait_for_keyboard_start()

    # Nombre de changements de fenêtre.
    # Exemple avec laps = 1 et p1,p2,p3,p4 :
    #   fenêtre 0 : p0 -> p1 -> p2
    #   switch p1
    #   fenêtre 1 : p1 -> p2 -> p3
    #   switch p2
    #   fenêtre 2 : p2 -> p3 -> p4
    #   switch p3
    #   fenêtre 3 : p3 -> p4 -> p1
    #   switch p4
    # Fin du tour.
    total_switches = laps * len(circuit_points)

    target_index = 0

    for switch_count in range(total_switches):
        if switch_count == 0:
            p_start = p0
            start_name = "p0"
        else:
            previous_index = (target_index - 1) % len(circuit_points)
            p_start = circuit_points[previous_index]
            start_name = circuit_names[previous_index]

        p_mid = circuit_points[target_index]
        mid_name = circuit_names[target_index]

        end_index = (target_index + 1) % len(circuit_points)
        p_end = circuit_points[end_index]
        end_name = circuit_names[end_index]

        logger.info(
            f"Planning window {switch_count + 1}/{total_switches}: "
            f"{start_name} -> {mid_name} -> {end_name}"
        )

        window_path = make_window_path(
            navigator=navigator,
            p_start=p_start,
            p_mid=p_mid,
            p_end=p_end,
            planner_id=planner_id,
            frame_id=frame_id,
            use_start_for_first_segment=use_start_for_first_segment,
        )

        if window_path is None or not window_path.poses:
            logger.error("Generated window path is empty")
            navigator.destroy_node()
            rclpy.shutdown()
            return

        logger.info(f"Window path poses: {len(window_path.poses)}")
        logger.info(f"Following until close to {mid_name}")

        navigator.followPath(
            path=window_path,
            controller_id=controller_id,
            goal_checker_id="general_goal_checker",
        )

        loop_count = 0

        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.05)
            loop_count += 1

            distance = navigator.distance_to_point(p_mid)

            if loop_count % log_freq == 0:
                logger.info(f"Distance to switch waypoint {mid_name}: {distance:.2f} m")

            if distance <= switch_distance:
                logger.info(
                    f"Switch waypoint reached: {mid_name}, "
                    f"distance={distance:.2f} m"
                )
                break

            if navigator.isTaskComplete():
                result = navigator.getResult()

                if result == TaskResult.SUCCEEDED:
                    logger.warn(
                        f"FollowPath finished before reaching switch distance for {mid_name}. "
                        f"distance={distance:.2f} m"
                    )
                    break

                if result == TaskResult.CANCELED:
                    logger.warn("FollowPath was canceled")
                    break

                if result == TaskResult.FAILED:
                    logger.error("FollowPath failed")
                    run_recovery(navigator, logger)
                    break

                logger.warn(f"FollowPath ended with unknown result={result}")
                break

            t.sleep(0.025)

        target_index = (target_index + 1) % len(circuit_points)

    logger.info("Race completed successfully")

    # Si une action FollowPath est encore active à la fin, on l'annule pour arrêter proprement.
    if not navigator.isTaskComplete():
        logger.info("Canceling final FollowPath task")
        navigator.cancelTask()
        t.sleep(0.1)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
