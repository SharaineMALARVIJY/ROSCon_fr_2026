import math
import time as t
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from ament_index_python.packages import get_package_share_directory
from race_path_follower.nav2_curve_anticipation import Nav2CurveAnticipation
from race_path_follower.lidar_recovery import LidarRecovery

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


def wait_for_task_result(navigator, logger, lap, goal_idx, curve_anticipation=None):
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.0)

        if curve_anticipation is not None:
            curve_anticipation.update()

        t.sleep(0.025)

    return navigator.getResult()


def run_recovery(recovery, navigator: BasicNavigator, goal_pose: PoseStamped) -> str:
    # On relance immédiatement le goal pendant la recovery.
    # Dès que Nav2 republie une commande sur /cmd_vel_nav, la recovery s'arrête.
    # La commande utilisée par la voiture reste /cmd_twist via le mux.
    recovery.reset_nav_cmd_watch()
    navigator.goToPose(goal_pose)
    return recovery.run(stop_on_nav_cmd=True)


def main() -> None:
    package_dir = get_package_share_directory('race_path_follower')
    rclpy.init(args=[
        '--ros-args',
        '--params-file',
        package_dir+'/params/race_params.yaml'
    ])

    navigator = BasicNavigator(node_name="race_node")
    logger = navigator.get_logger()

    # ========================
    # Paramètres ROS
    # ========================
    navigator.declare_parameter("laps", 6)
    navigator.declare_parameter("max_retries", 30)
    # Paramètres curve anticipation: ralentissement anticipé avant les virages
    navigator.declare_parameter("curve_plan_topic", "/plan")
    navigator.declare_parameter("curve_odom_topic", "/odom")
    navigator.declare_parameter("curve_controller_server", "/controller_server")

    navigator.declare_parameter("curve_speed_param_name", "FollowPath.desired_linear_vel")
    navigator.declare_parameter("curve_lookahead_param_name", "FollowPath.max_lookahead_dist")
    navigator.declare_parameter("curve_regulated_min_speed_param_name", "FollowPath.regulated_linear_scaling_min_speed")
    navigator.declare_parameter("curve_change_regulated_min_speed", True)

    navigator.declare_parameter("curve_straight_speed", 2.5)
    navigator.declare_parameter("curve_normal_turn_speed", 1.6)
    navigator.declare_parameter("curve_sharp_turn_speed", 1.1)

    navigator.declare_parameter("curve_straight_max_lookahead_dist", 1.2)
    navigator.declare_parameter("curve_normal_turn_max_lookahead_dist", 0.8)
    navigator.declare_parameter("curve_sharp_turn_max_lookahead_dist", 0.65)

    # Fenêtre du /plan analysée devant la voiture.
    # On ignore le début [0, start] pour éviter les petites corrections locales,
    # puis on analyse seulement [start, end].
    navigator.declare_parameter("curve_path_start_distance", 0.5)
    navigator.declare_parameter("curve_path_end_distance", 3.0)
    navigator.declare_parameter("curve_path_nearest_search_max_points", 80)
    navigator.declare_parameter("curve_turn_heading_change_deg", 18.0)
    navigator.declare_parameter("curve_sharp_turn_heading_change_deg", 35.0)

    navigator.declare_parameter("curve_min_param_update_period", 0.20)
    navigator.declare_parameter("curve_min_mode_duration", 0.35)
    navigator.declare_parameter("curve_log_period", 0.50)

    navigator.declare_parameter("tracks_file", package_dir+"/params/tracks.yaml")
    navigator.declare_parameter("active_track", "")

    laps = int(navigator.get_parameter("laps").value)
    max_retries = int(navigator.get_parameter("max_retries").value)

    tracks_file = str(navigator.get_parameter("tracks_file").value)
    active_track = str(navigator.get_parameter("active_track").value)

    curve_plan_topic = str(navigator.get_parameter("curve_plan_topic").value)
    curve_odom_topic = str(navigator.get_parameter("curve_odom_topic").value)
    curve_controller_server = str(navigator.get_parameter("curve_controller_server").value)

    curve_speed_param_name = str(navigator.get_parameter("curve_speed_param_name").value)
    curve_lookahead_param_name = str(navigator.get_parameter("curve_lookahead_param_name").value)
    curve_regulated_min_speed_param_name = str(
        navigator.get_parameter("curve_regulated_min_speed_param_name").value
    )
    curve_change_regulated_min_speed = bool(
        navigator.get_parameter("curve_change_regulated_min_speed").value
    )

    curve_straight_speed = float(navigator.get_parameter("curve_straight_speed").value)
    curve_normal_turn_speed = float(navigator.get_parameter("curve_normal_turn_speed").value)
    curve_sharp_turn_speed = float(navigator.get_parameter("curve_sharp_turn_speed").value)

    curve_straight_max_lookahead_dist = float(
        navigator.get_parameter("curve_straight_max_lookahead_dist").value
    )
    curve_normal_turn_max_lookahead_dist = float(
        navigator.get_parameter("curve_normal_turn_max_lookahead_dist").value
    )
    curve_sharp_turn_max_lookahead_dist = float(
        navigator.get_parameter("curve_sharp_turn_max_lookahead_dist").value
    )

    curve_path_start_distance = float(
        navigator.get_parameter("curve_path_start_distance").value
    )
    curve_path_end_distance = float(
        navigator.get_parameter("curve_path_end_distance").value
    )
    curve_path_nearest_search_max_points = int(
        navigator.get_parameter("curve_path_nearest_search_max_points").value
    )
    curve_turn_heading_change_deg = float(
        navigator.get_parameter("curve_turn_heading_change_deg").value
    )
    curve_sharp_turn_heading_change_deg = float(
        navigator.get_parameter("curve_sharp_turn_heading_change_deg").value
    )

    curve_min_param_update_period = float(
        navigator.get_parameter("curve_min_param_update_period").value
    )
    curve_min_mode_duration = float(
        navigator.get_parameter("curve_min_mode_duration").value
    )
    curve_log_period = float(navigator.get_parameter("curve_log_period").value)

    recovery = LidarRecovery(navigator)

    curve_anticipation = Nav2CurveAnticipation(
        node=navigator,
        logger=logger,
        plan_topic=curve_plan_topic,
        odom_topic=curve_odom_topic,
        controller_server=curve_controller_server,

        speed_param_name=curve_speed_param_name,
        lookahead_param_name=curve_lookahead_param_name,
        regulated_min_speed_param_name=curve_regulated_min_speed_param_name,
        change_regulated_min_speed=curve_change_regulated_min_speed,

        straight_speed=curve_straight_speed,
        normal_turn_speed=curve_normal_turn_speed,
        sharp_turn_speed=curve_sharp_turn_speed,

        straight_max_lookahead_dist=curve_straight_max_lookahead_dist,
        normal_turn_max_lookahead_dist=curve_normal_turn_max_lookahead_dist,
        sharp_turn_max_lookahead_dist=curve_sharp_turn_max_lookahead_dist,

        path_start_distance=curve_path_start_distance,
        path_end_distance=curve_path_end_distance,
        path_nearest_search_max_points=curve_path_nearest_search_max_points,
        turn_heading_change_deg=curve_turn_heading_change_deg,
        sharp_turn_heading_change_deg=curve_sharp_turn_heading_change_deg,

        min_param_update_period=curve_min_param_update_period,
        min_mode_duration=curve_min_mode_duration,
        log_period=curve_log_period,
    )

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
                result = wait_for_task_result(navigator, logger, lap, goal_idx, curve_anticipation)

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

                    recovery_status = run_recovery(recovery, navigator, goal_pose)
                    logger.info(f"Recovery status: {recovery_status}")

                    if recovery_status == "NAV_CMD_ACTIVE":
                        result_after_recovery = wait_for_task_result(
                            navigator, logger, lap, goal_idx, curve_anticipation
                        )

                        if result_after_recovery == TaskResult.SUCCEEDED:
                            logger.info(f"Lap {lap}: {goal_name} succeeded after recovery")
                            success = True
                            break

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

    recovery.stop()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()