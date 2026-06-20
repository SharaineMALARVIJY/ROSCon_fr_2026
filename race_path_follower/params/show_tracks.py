import math
import yaml
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from PIL import Image

# =========================
# Paramètres
# =========================
TRACKS_FILE = "tracks.yaml"

# "" = utilise active_track dans tracks.yaml
ACTIVE_TRACK = ""

# Si None, le nom de map est le même que la ACTIVE_TRACK,
# avec suppression de "_reverse" si présent.
# Exemple : MAP_NAME_OVERRIDE = "st_cyr_v3" pour forcer une map précise
MAP_NAME_OVERRIDE = None

MAP_FOLDER = "../maps"







MAP_IMAGE_EXTENSION = "pgm"
ROTATE_MAP_IMAGE = False

ARROW_LENGTH_M = 0.5
POINT_MARKER_SIZE = 6
LABEL_FONT_SIZE = 10
LABEL_OFFSET_PX = 5
ARROW_HEAD_WIDTH_PX = 5
ARROW_HEAD_LENGTH_PX = 5

GOAL_TOLERANCE_RADIUS_M = 1.5
GOAL_TOLERANCE_LABEL = "goal tolerance"
GOAL_TOLERANCE_COLOR = "orange"
GOAL_TOLERANCE_LINE_WIDTH = 1.5

POINT_COLOR = "red"
LABEL_COLOR = "red"
ARROW_COLOR = "blue"

def get_resolution_origin(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    return data["resolution"], data["origin"]


def open_image(image_path, rotate=False):
    img = Image.open(image_path)

    if rotate:
        img = np.rot90(img)

    return np.array(img, dtype=np.float32)


def coord_to_pix(x, y, resolution, origin, height):
    pix_x = (x - origin[0]) / resolution
    pix_y = height - ((y - origin[1]) / resolution)

    return pix_x, pix_y


def get_map_name_from_track_name(active_track):
    if active_track.endswith("_reverse"):
        return active_track.removesuffix("_reverse")

    return active_track


def load_track_points(tracks_file, active_track=""):
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
                "Ajoute active_track dans tracks.yaml ou donne un nom dans affichage.py."
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
            f"La trajectoire '{active_track}' doit contenir au moins 2 points."
        )

    for i, p in enumerate(points):
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(
                f"Point p{i} invalide. Format attendu : [x, y, yaw]."
            )

        float(p[0])
        float(p[1])
        float(p[2])

    return active_track, points


def draw_points_and_orientations(
    image,
    resolution,
    origin,
    points,
    active_track,
    arrow_length_m=ARROW_LENGTH_M,
    point_marker_size=POINT_MARKER_SIZE,
    label_font_size=LABEL_FONT_SIZE,
    label_offset_px=LABEL_OFFSET_PX,
    arrow_head_width_px=ARROW_HEAD_WIDTH_PX,
    arrow_head_length_px=ARROW_HEAD_LENGTH_PX,
    goal_tolerance_radius_m=GOAL_TOLERANCE_RADIUS_M,
    goal_tolerance_label=GOAL_TOLERANCE_LABEL,
    goal_tolerance_color=GOAL_TOLERANCE_COLOR,
    goal_tolerance_line_width=GOAL_TOLERANCE_LINE_WIDTH,
    point_color=POINT_COLOR,
    label_color=LABEL_COLOR,
    arrow_color=ARROW_COLOR,
):
    height, width = image.shape
    labels = [f"p{i}" for i in range(len(points))]

    fig, ax = plt.subplots()
    ax.imshow(image, cmap="gray")

    for i, (x, y, theta) in enumerate(points):
        px, py = coord_to_pix(x, y, resolution, origin, height)

        if i != 0 and goal_tolerance_radius_m > 0:
            goal_tolerance_circle = plt.Circle(
                (px, py),
                goal_tolerance_radius_m / resolution,
                fill=False,
                color=goal_tolerance_color,
                linewidth=goal_tolerance_line_width,
                label=goal_tolerance_label if i == 1 else None,
            )
            ax.add_patch(goal_tolerance_circle)

        ax.plot(px, py, "o", color=point_color, markersize=point_marker_size)
        ax.text(
            px + label_offset_px,
            py + label_offset_px,
            labels[i],
            color=label_color,
            fontsize=label_font_size,
        )

        dx_m = arrow_length_m * math.cos(theta)
        dy_m = arrow_length_m * math.sin(theta)

        px2, py2 = coord_to_pix(
            x + dx_m,
            y + dy_m,
            resolution,
            origin,
            height,
        )

        ax.arrow(
            px,
            py,
            px2 - px,
            py2 - py,
            head_width=arrow_head_width_px,
            head_length=arrow_head_length_px,
            fc=arrow_color,
            ec=arrow_color,
        )

    ax.set_title(f"Trajectoire : {active_track}")
    ax.set_aspect("equal")

    if goal_tolerance_radius_m > 0 and len(points) > 1:
        ax.legend()
    plt.show()


def main():
    active_track, points = load_track_points(
        tracks_file=TRACKS_FILE,
        active_track=ACTIVE_TRACK,
    )

    if MAP_NAME_OVERRIDE is None:
        map_name = get_map_name_from_track_name(active_track)
    else:
        map_name = MAP_NAME_OVERRIDE

    map_yaml_path = f"{MAP_FOLDER}/{map_name}.yaml"
    map_image_path = f"{MAP_FOLDER}/{map_name}.{MAP_IMAGE_EXTENSION}"

    print(f"Trajectoire chargée : {active_track}")
    print(f"Nom de map utilisé : {map_name}")
    print(f"Map YAML : {map_yaml_path}")
    print(f"Map image : {map_image_path}")
    print(f"Nombre de points : {len(points)}")

    for i, p in enumerate(points):
        print(f"p{i}: {p}")

    image = open_image(map_image_path, rotate=ROTATE_MAP_IMAGE)
    resolution, origin = get_resolution_origin(map_yaml_path)

    draw_points_and_orientations(
        image=image,
        resolution=resolution,
        origin=origin,
        points=points,
        active_track=active_track,
        arrow_length_m=ARROW_LENGTH_M,
        goal_tolerance_radius_m=GOAL_TOLERANCE_RADIUS_M,
    )


if __name__ == "__main__":
    main()
