import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def plot_transform(ax, T, label, length=0.2):
    """Draw a coordinate frame using a 4x4 transformation matrix."""
    # origin = T[:3, 3]
    # x_axis = T[:3, 0] * length
    # y_axis = T[:3, 1] * length
    # z_axis = T[:3, 2] * length
    H = np.linalg.inv(T)
    origin = (H @ np.array([0,0,0,1]))[:3]
    x_axis = (H @ np.array([1,0,0,1]))[:3]
    y_axis = (H @ np.array([0,1,0,1]))[:3]
    z_axis = (H @ np.array([0,0,1,1]))[:3]

    print(f"{origin=}")

    ax.quiver(*origin, *(x_axis-origin) * length, color='r')
    ax.quiver(*origin, *(y_axis-origin) * length, color='g')
    ax.quiver(*origin, *(z_axis-origin) * length, color='b')
    # Place label at the tip of +X axis
    tip_x = origin + ((x_axis-origin)*length)
    ax.text(*tip_x, label, fontsize=8)


def set_axes_equal(ax):
    """Make 3D plot axes have equal scale."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def main():
    parser = argparse.ArgumentParser(description="Plot coordinate transforms from a JSON file.")
    parser.add_argument("json_file", help="Path to JSON file containing transforms")
    args = parser.parse_args()

    with open(args.json_file, 'r') as f:
        transforms = json.load(f)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    IGNORE = ["T_body_to_imu", "T_body_to_decawave", "T_imu_to_sbody"]
    for name, mat in transforms.items():
        if "T_world_to" in name or (name == "origin") or (name =="T_sorigin_to_sbody"):
            T = np.array(mat)
            print(f"{name=}")
            print(T)
            plot_transform(ax, T, name)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Coordinate Frames")
#    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=20, azim=45)

    set_axes_equal(ax)
    plt.show()

if __name__ == "__main__":
    main()
