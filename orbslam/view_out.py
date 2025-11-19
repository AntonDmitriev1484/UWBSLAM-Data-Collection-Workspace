#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (required for 3D projection)
from scipy.spatial.transform import Rotation as R
import os


def draw_axes(ax, T, length=1):
    """Draw coordinate axes for transformation matrix T."""
    origin = T[:3, 3]
    rot = T[:3, :3]

    # X (red), Y (green), Z (blue)
    ax.quiver(*origin, *rot[:, 0] * length, color='r' )
    ax.quiver(*origin, *rot[:, 1] * length, color='g')
    ax.quiver(*origin, *rot[:, 2] * length, color='b')


def load_trajectory(file_path):
    """Load whitespace-separated trajectory file into Nx8 array."""
    data = np.loadtxt(file_path)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quats = data[:, 4:8]  # (x, y, z, w)
    return timestamps, positions, quats


def plot_trajectory_with_axes(timestamps, positions, quats, stride=10):
    """Plot 3D trajectory and coordinate axes for each pose."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    ax.plot(x, y, z, color='green', linewidth=2, label='Trajectory')

    for i in range(0, len(positions), stride):
        # Convert quaternion to rotation matrix
        R_mat = R.from_quat(quats[i]).as_matrix()

        # Build homogeneous transform
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = positions[i]

        draw_axes(ax, T, length=0.3)

    ax.scatter(x[0], y[0], z[0], color='lime', s=50, label='Start')
    ax.scatter(x[-1], y[-1], z[-1], color='blue', s=50, label='End')

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(0,3)
    ax.set_title("3D Pose Trajectory with Coordinate Axes")
    ax.legend()
    ax.grid(True)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot 3D trajectory with pose axes")
    parser.add_argument("--trial", "-t", required=True, help="Trial name")
    parser.add_argument("--kf", action="store_true")
    parser.add_argument("--stride", type=int, default=10, help="Plot every Nth pose for clarity")
    args = parser.parse_args()

    traj_path = f"./out/{args.trial}"
    if args.kf:
        traj_path = f"./out/{args.trial}_kf_traj.txt"
    else:
        traj_path = f"./out/{args.trial}_cam_traj.txt"
    
    if not os.path.exists(traj_path):
        print(f"Error: trajectory file not found: {traj_path}")
        exit(1)

    timestamps, positions, quats = load_trajectory(traj_path)
    plot_trajectory_with_axes(timestamps, positions, quats, stride=args.stride)


if __name__ == "__main__":
    main()
