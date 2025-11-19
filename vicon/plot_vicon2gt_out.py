#!/usr/bin/env python3
import argparse
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation as R


def load_vicon2gt_states(csv_path, stride):
    """Load CSV manually (handles # headers) and subsample."""
    data = []
    headers = None
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # Header line starts with "#"
            if row[0].startswith("#"):
                headers = [h.strip() for h in row[0][1:].split(",")]
                continue
            if headers is None:
                headers = [h.strip() for h in row]
                continue
            data.append([float(x) for x in row])
    data = np.array(data)
    return data


def draw_axes(ax, T, length=1):
    """Draw coordinate frame axes given a 4x4 transform T."""
    # H = np.linalg.inv(T)  # Draw in world frame convention
    H = T # Should already be body to world transform
    origin = (H @ np.array([0, 0, 0, 1]))[:3]
    x_axis = (H @ np.array([1, 0, 0, 1]))[:3]
    y_axis = (H @ np.array([0, 1, 0, 1]))[:3]
    z_axis = (H @ np.array([0, 0, 1, 1]))[:3]

    ax.quiver(*origin, *(x_axis - origin) * length, color='r')
    ax.quiver(*origin, *(y_axis - origin) * length, color='g')
    ax.quiver(*origin, *(z_axis - origin) * length, color='b')


def plot_vicon2gt_trajectory(data, trial_name, stride):
    """Plot trajectory and pose frames in 3D following the structured format."""

    # Something inverted here, either in the data or in the plotting.

    # Convert to meters

    # Build 4x4 pose matrices
    poses = [] # T _body_to_world
    positions = []
    for i in range(data.shape[0]):
        T = np.eye(4)
        t_world_to_body = data[i, 1:4] /1e3
        quat = data[i, 4:8]
        R_body_to_world = R.from_quat(quat).as_matrix()
        T[:3, :3] = R_body_to_world
        T[:3, 3] = t_world_to_body
        positions.append(t_world_to_body)
        poses.append(T)

    # Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')


    positions = np.array(positions)
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            label='Vicon2GT Trajectory', color='royalblue', linewidth=2)
    ax.scatter(*positions[0], color='green', label='Start')
    ax.scatter(*positions[-1], color='red', label='End')

    # Draw coordinate frames along the path
    if stride > 0:
        for i in range(0, len(poses), stride):
            draw_axes(ax, poses[i], length=0.4)

    # Axis labels & formatting
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_zlim(-2,2)
    ax.set_title(f"Vicon2GT Trajectory — {trial_name}")
    ax.legend()
    ax.grid(True)
    # ax.view_init(elev=25, azim=45)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot 3D Vicon2GT trajectory and coordinate frames.")
    parser.add_argument("-t", "--trial_name", required=True, help="Trial name (e.g. 'irl4_free_together')")
    parser.add_argument("--stride", type=int, default=20, help="Stride for drawing coordinate frames (default: 20)")
    args = parser.parse_args()

    csv_path = f"vicon2gt/out/{args.trial_name}_vicon2gt_states.csv"
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Missing file: {csv_path}")

    data = load_vicon2gt_states(csv_path, stride=1)
    plot_vicon2gt_trajectory( data, args.trial_name, args.stride)


if __name__ == "__main__":
    main()
