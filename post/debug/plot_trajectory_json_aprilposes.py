import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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

def draw_axes(ax, T, length=0.1):
    """Draw coordinate axes from transformation matrix T."""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)

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


def main():
    parser = argparse.ArgumentParser(description="Plot trajectory and coordinate transforms from all.json")
    parser.add_argument("all_json", help="Path to all.json file")
    parser.add_argument("--stride", type=int, default=0, help="Stride to draw trajectory axes (default: 20)")
    parser.add_argument("--transforms_json", help="Optional transforms.json file", default=None)
    args = parser.parse_args()

    with open(args.all_json, 'r') as f:
        all_data = json.load(f)

    transforms = []
    positions = []

    april_positions = []
    april_transforms = []

    for item in all_data:
        if item.get("type") == "slam_pose" and "T_body_world" in item:
            T = np.array(item["T_body_world"]) # T_world_to_body
            transforms.append(T)
            H = np.linalg.inv(T)
            positions.append(H[:3, 3])
        if item.get("type") == "slam_pose" and "April_T_body_world" in item:
            if item["April_T_body_world"] is not None:
                Ts = item["April_T_body_world"]
                for T in Ts:
                    T = np.array(T)
                    H = np.linalg.inv(T)
                    april_transforms.append(T)
                    april_positions.append(H[:3, 3])

    positions = np.array(positions)
    april_positions = np.array(april_positions)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory', color='blue')
    ax.scatter(april_positions[:, 0], april_positions[:, 1], april_positions[:, 2], label='April Poses', color='purple')
    ax.scatter(*positions[0], color='green', label='Start')
    ax.scatter(*positions[-1], color='red', label='End')

    for T in april_transforms:
        plot_transform(ax, T, 'A', length = 0.4)

    # if args.stride > 0:
    #     for i in range(0, len(transforms), args.stride):
    #         # draw_axes(ax, transforms[i], length=0.4)

    # Optional: load static transforms from separate transforms.json
    if args.transforms_json:
        with open(args.transforms_json, 'r') as f:
            static_transforms = json.load(f)
        for name, mat in static_transforms.items():
            T = np.array(mat)
            plot_transform(ax, T, name)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory and Static Coordinate Frames")
    ax.view_init(elev=20, azim=45)
    set_axes_equal(ax)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
