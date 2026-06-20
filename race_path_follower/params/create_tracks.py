import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import yaml
from PIL import Image


class FlowList(list):
    pass


def flow_list_representer(dumper, data):
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq",
        data,
        flow_style=True,
    )


yaml.add_representer(FlowList, flow_list_representer)


def get_resolution_origin(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    return data["resolution"], data["origin"]


def open_image(image_path, rotate=False):
    img = Image.open(image_path)

    if rotate:
        img = np.rot90(img)

    return np.array(img, dtype=np.float32)


def pix_to_coord(pix_x, pix_y, resolution, origin, height):
    x = origin[0] + pix_x * resolution
    y = origin[1] + (height - pix_y) * resolution
    return x, y


def coord_to_pix(x, y, resolution, origin, height):
    pix_x = (x - origin[0]) / resolution
    pix_y = height - ((y - origin[1]) / resolution)
    return pix_x, pix_y


def get_map_name_from_track_name(active_track):
    if active_track.endswith("_reverse"):
        return active_track.removesuffix("_reverse")

    return active_track


def load_tracks_file(tracks_file):
    try:
        with open(tracks_file, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        data = None

    if data is None:
        data = {
            "active_track": "",
            "tracks": {},
        }

    if "tracks" not in data:
        data["tracks"] = {}

    if "active_track" not in data:
        data["active_track"] = ""

    return data


def get_active_track(tracks_file):
    data = load_tracks_file(tracks_file)

    active_track = data.get("active_track", "")

    if active_track == "":
        active_track = input("Nom de la trajectoire : ").strip()

        if active_track == "":
            raise ValueError("Nom de trajectoire vide.")

    return active_track


def to_vect(clicked_points):
    if len(clicked_points) % 2 != 0:
        return None

    vects = []

    for i in range(0, len(clicked_points), 2):
        a = clicked_points[i]
        b = clicked_points[i + 1]

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        theta = float(np.arctan2(dy, dx))

        vects.append([
            float(a[0]),
            float(a[1]),
            theta,
        ])

    return vects


def print_course(vects):
    if not vects:
        print("Aucun vecteur à afficher.")
        return

    print("\nPoints créés :\n")

    for i, p in enumerate(vects):
        print(f"p{i}: {p}")


def round_point(p, ndigits=4):
    return [
        round(float(p[0]), ndigits),
        round(float(p[1]), ndigits),
        round(float(p[2]), ndigits),
    ]


def format_all_points_for_yaml(data):
    if "tracks" not in data:
        return data

    for track_name, track_data in data["tracks"].items():
        if "points" not in track_data:
            continue

        formatted_points = []

        for p in track_data["points"]:
            formatted_points.append(
                FlowList(round_point(p, ndigits=4))
            )

        track_data["points"] = formatted_points

    return data


def save_track(tracks_file, active_track, vects):
    data = load_tracks_file(tracks_file)

    data["active_track"] = active_track

    data["tracks"][active_track] = {
        "points": [round_point(p, ndigits=4) for p in vects],
    }

    data = format_all_points_for_yaml(data)

    with open(tracks_file, "w") as f:
        yaml.dump(
            data,
            f,
            sort_keys=False,
            default_flow_style=False,
            allow_unicode=True,
            width=1000,
        )

    print(f"\nTrajectoire sauvegardée dans {tracks_file}")
    print(f"Trajectoire active : {active_track}")


def redraw(ax, image, resolution, origin, height, active_track):
    ax.clear()
    ax.imshow(image, cmap="gray")

    ax.set_title(
        f"Création trajectoire : {active_track}\n"
        "clic gauche = ajouter | clic droit = annuler | fermer = terminer"
    )

    arrow_length_m = 0.5

    vects = to_vect(x_y_reals)

    if vects is not None:
        for i, (x, y, theta) in enumerate(vects):
            px, py = coord_to_pix(x, y, resolution, origin, height)

            ax.plot(px, py, "ro", markersize=6)
            ax.text(px + 5, py + 5, f"p{i}", color="red", fontsize=10)

            dx_m = arrow_length_m * np.cos(theta)
            dy_m = arrow_length_m * np.sin(theta)

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
                head_width=5,
                head_length=5,
                fc="blue",
                ec="blue",
            )

    if len(x_y_reals) % 2 == 1:
        point_id = len(x_y_reals) // 2
        x, y = x_y_reals[-1]

        px, py = coord_to_pix(x, y, resolution, origin, height)

        ax.plot(px, py, "yo", markersize=7)
        ax.text(px + 5, py + 5, f"p{point_id} ?", color="yellow", fontsize=10)

    ax.set_aspect("equal")
    plt.draw()


def onclick(event, ax, image, resolution, origin, height, active_track):
    if event.xdata is None or event.ydata is None:
        return

    if event.button == 1:
        px = int(event.xdata)
        py = int(event.ydata)

        real_point = pix_to_coord(px, py, resolution, origin, height)
        x_y_reals.append(real_point)

        click_id = len(x_y_reals)

        if click_id % 2 == 1:
            point_id = click_id // 2
            print(f"p{point_id} position : {real_point}")
        else:
            point_id = click_id // 2 - 1
            vects = to_vect(x_y_reals)
            print(f"p{point_id} complet : {vects[-1]}")

    elif event.button == 3:
        if x_y_reals:
            removed = x_y_reals.pop()
            print(f"Dernier clic supprimé : {removed}")

    redraw(ax, image, resolution, origin, height, active_track)


def main():
    tracks_file = "tracks.yaml"

    active_track = get_active_track(tracks_file)
    map_name = get_map_name_from_track_name(active_track)

    yaml_path = f"../maps/{map_name}.yaml"
    image_path = f"../maps/{map_name}.pgm"

    print(f"Trajectoire active : {active_track}")
    print(f"Nom de map utilisé : {map_name}")
    print(f"Map YAML : {yaml_path}")
    print(f"Map image : {image_path}")

    print("\nUtilisation :")
    print("- clic gauche 1 : position du point")
    print("- clic gauche 2 : direction du point")
    print("- clic droit : annuler le dernier clic")
    print("- fermer la fenêtre quand tous les points sont faits\n")

    image = open_image(image_path, False)
    resolution, origin = get_resolution_origin(yaml_path)
    height, width = image.shape

    fig, ax = plt.subplots()
    redraw(ax, image, resolution, origin, height, active_track)

    fig.canvas.mpl_connect(
        "button_press_event",
        lambda event: onclick(
            event,
            ax,
            image,
            resolution,
            origin,
            height,
            active_track,
        ),
    )

    plt.show()

    vects = to_vect(x_y_reals)

    if vects is None:
        print("Erreur : nombre de clics impair.")
        print("Il faut 2 clics par point : position puis orientation.")
        return

    if len(vects) < 2:
        print("Erreur : il faut au moins 2 points.")
        return

    print_course(vects)

    save = input("\nSauvegarder cette trajectoire ? [o/N] : ").strip().lower()

    if save == "o":
        save_track(tracks_file, active_track, vects)
    else:
        print("Trajectoire non sauvegardée.")


x_y_reals = []


if __name__ == "__main__":
    main()
