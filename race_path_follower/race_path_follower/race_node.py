import math
import time as t
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(navigator: BasicNavigator, p, frame_id: str = "map") -> PoseStamped:
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
    except Exception:
        logger.error(f"{name} values must be numbers: [x, y, yaw]")
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

    points = track_data["points"]

    if not isinstance(points, list):
        raise ValueError(
            f"Les points de la trajectoire '{active_track}' doivent être une liste."
        )

    if len(points) < 2:
        raise ValueError(
            f"La trajectoire '{active_track}' doit contenir au moins 2 points : "
            f"p0 + au moins un objectif."
        )

    return active_track, points


def wait_for_keyboard_start() -> None:
    while True:
        key = input("Tape 'd' puis Entrée pour démarrer : ").strip().lower()
        if key == "d":
            return
        print("Commande invalide. Tape seulement 'd'.")


def wait_for_task_result(navigator: BasicNavigator, logger, lap: int, goal_idx: int):
    while not navigator.isTaskComplete():
        # feedback = navigator.getFeedback()
        #
        # if feedback:
        #     distance = getattr(feedback, "distance_remaining", None)
        #     if distance is not None:
        #         logger.info(
        #             f"Lap {lap}: goal {goal_idx}: remaining {distance:.2f} m",
        #             throttle_duration_sec=1.0
        #         )
        t.sleep(0.025)

    return navigator.getResult()


def run_recovery(navigator: BasicNavigator, logger) -> None:
    logger.warn("Recovery: backup")

    # disable_collision_checks n'existe pas sur Jazzy ce qui rend impossible la marche arrière de la voiture si elle croit etre dans un mur
    navigator.backup(
        backup_dist=0.3,
        backup_speed=0.5,
        time_allowance=2
    )

    while not navigator.isTaskComplete():
        t.sleep(0.025)

    recovery_result = navigator.getResult()
    logger.info(f"Recovery result: {recovery_result}")


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator(node_name="race_node")
    logger = navigator.get_logger()

    # ========================
    # Paramètres ROS
    # ========================
    navigator.declare_parameter("laps", 1)
    navigator.declare_parameter("max_retries", 30)

    # Nouveau système propre
    navigator.declare_parameter("tracks_file", "tracks.yaml")
    navigator.declare_parameter("active_track", "")

    laps = int(navigator.get_parameter("laps").value)
    max_retries = int(navigator.get_parameter("max_retries").value)

    tracks_file = str(navigator.get_parameter("tracks_file").value)
    active_track = str(navigator.get_parameter("active_track").value)

    # ========================
    # Chargement des points
    # ========================
    try:
        active_track, points = load_track_points(tracks_file, active_track)
    except Exception as e:
        logger.error(f"Erreur chargement trajectoire : {e}")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    p0 = points[0]
    goal_points_raw = points[1:]

    if not validate_point(logger, "p0", p0):
        navigator.destroy_node()
        rclpy.shutdown()
        return

    goal_points_one_lap = []

    for i, p in enumerate(goal_points_raw, start=1):
        name = f"p{i}"

        if not validate_point(logger, name, p):
            navigator.destroy_node()
            rclpy.shutdown()
            return

        goal_points_one_lap.append((name, make_pose(navigator, p)))

    if not goal_points_one_lap:
        logger.error("No active goal points. The track must contain at least p0 and p1.")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # ========================
    # Vérification paramètres
    # ========================
    if laps < 1:
        logger.error("laps must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if max_retries < 1:
        logger.error("max_retries must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # ========================
    # Logs
    # ========================
    logger.info(f"Tracks file: {tracks_file}")
    logger.info(f"Active track: {active_track}")
    logger.info(f"Initial pose p0: {p0}")
    logger.info(f"Active goals per lap: {len(goal_points_one_lap)}")
    logger.info(f"Total goals over all laps: {len(goal_points_one_lap) * laps}")
    logger.info(f"Max retries per goal: {max_retries}")

    # ========================
    # Initialisation Nav2
    # ========================
    navigator.setInitialPose(make_pose(navigator, p0))

    logger.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    logger.info("Nav2 ready")

    wait_for_keyboard_start()

    # ========================
    # Navigation
    # ========================
    for lap in range(1, laps + 1):
        logger.info(f"Starting lap {lap}/{laps}")

        for goal_idx, (goal_name, goal_pose) in enumerate(goal_points_one_lap, start=1):
            success = False

            for attempt in range(1, max_retries + 1):
                logger.info(
                    f"Lap {lap}: sending {goal_name} "
                    f"({goal_idx}/{len(goal_points_one_lap)}) "
                    f"attempt {attempt}/{max_retries}"
                )

                navigator.goToPose(goal_pose)
                result = wait_for_task_result(navigator, logger, lap, goal_idx)

                if result == TaskResult.SUCCEEDED:
                    logger.info(f"Lap {lap}: {goal_name} succeeded")
                    success = True
                    break

                if result == TaskResult.CANCELED:
                    logger.warn(
                        f"Lap {lap}: {goal_name} canceled "
                        f"({attempt}/{max_retries})"
                    )

                elif result == TaskResult.FAILED:
                    logger.warn(
                        f"Lap {lap}: {goal_name} failed "
                        f"({attempt}/{max_retries})"
                    )
                    run_recovery(navigator, logger)

                else:
                    logger.warn(
                        f"Lap {lap}: {goal_name} unknown result={result} "
                        f"({attempt}/{max_retries})"
                    )

            if not success:
                logger.error(
                    f"Lap {lap}: {goal_name} reached max retries "
                    f"({max_retries}), skipping"
                )

    logger.info("All laps completed.")

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()