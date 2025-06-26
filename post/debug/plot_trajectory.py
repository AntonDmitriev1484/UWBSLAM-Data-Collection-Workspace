import csv
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
    """Draw a coordinate frame using a 4x4 transformation matrix and label at +X tip."""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r')
    ax.quiver(*origin, *y_axis, color='g')
    ax.quiver(*origin, *z_axis, color='b')

    tip_x = origin + x_axis
    ax.text(*tip_x, label, fontsize=8)

def main():
    parser = argparse.ArgumentParser(description="Plot trajectory and coordinate transforms together.")
    parser.add_argument("csv_file", help="CSV file with trajectory data")
    parser.add_argument("json_file", help="JSON file with static transforms")
    parser.add_argument("--stride", type=int, default=20, help="Stride to draw trajectory axes (default: 20)")
    args = parser.parse_args()

    timestamps = []
    positions = []
    transforms = []

    # --- Load CSV trajectory ---
    with open(args.csv_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if len(row) != 17:
                print(f"Skipping malformed row: {row}")
                continue
            timestamp = float(row[0])
            T_flat = list(map(float, row[1:]))
            T = np.array(T_flat).reshape((4, 4))
            timestamps.append(timestamp)
            positions.append(T[:3, 3])
            transforms.append(T)

    positions = np.array(positions)

    # --- Load JSON transforms ---
    with open(args.json_file, 'r') as f:
        static_transforms = json.load(f)

    # --- Plot all together ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory', color='blue')
    ax.scatter(*positions[0], color='green', label='Start')
    ax.scatter(*positions[-1], color='red', label='End')

    for i in range(0, len(transforms), args.stride):
        draw_axes(ax, transforms[i], length=0.4)

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


# import csv
# import argparse
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# def set_axes_equal(ax):
#     """Make 3D plot axes have equal scale."""
#     x_limits = ax.get_xlim3d()
#     y_limits = ax.get_ylim3d()
#     z_limits = ax.get_zlim3d()

#     x_range = abs(x_limits[1] - x_limits[0])
#     x_middle = np.mean(x_limits)
#     y_range = abs(y_limits[1] - y_limits[0])
#     y_middle = np.mean(y_limits)
#     z_range = abs(z_limits[1] - z_limits[0])
#     z_middle = np.mean(z_limits)

#     plot_radius = 0.5 * max([x_range, y_range, z_range])
#     ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
#     ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
#     ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# def draw_axes(ax, T, length=0.1):
#     """Draw coordinate axes from transformation matrix T."""
#     origin = T[:3, 3]
#     x_axis = T[:3, 0] * length
#     y_axis = T[:3, 1] * length
#     z_axis = T[:3, 2] * length

#     ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
#     ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
#     ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)

# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("csv_file", help="CSV file with trajectory data")
#     args = parser.parse_args()

#     timestamps = []
#     positions = []
#     transforms = []

#     with open(args.csv_file, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         for row in reader:
#             if len(row) != 17:
#                 print(f"Skipping malformed row: {row}")
#                 continue

#             timestamp = float(row[0])
#             T_flat = list(map(float, row[1:]))
#             T = np.array(T_flat).reshape((4, 4))
#             position = T[:3, 3]

#             timestamps.append(timestamp)
#             positions.append(position)
#             transforms.append(T)

#     positions = np.array(positions)
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # Plot trajectory line
#     ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory', color='blue')
#     ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], color='green', label='Start')
#     ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], color='red', label='End')

#     # Plot coordinate axes every 20th transform
#     for i in range(0, len(transforms), 20):
#         draw_axes(ax, transforms[i], length=0.4)

#     ax.set_xlabel("X")
#     ax.set_ylabel("Y")
#     ax.set_zlabel("Z")
#     ax.set_title("Trajectory with Orientation Axes")
#     set_axes_equal(ax)
#     ax.legend()
#     plt.show()

# if __name__ == "__main__":
#     main()
