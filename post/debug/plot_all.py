import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def draw_axes(ax, T, length=0.1):
    """Draw coordinate axes from transformation matrix T."""
    H = np.linalg.inv(T)
    origin = (H @ np.array([0,0,0,1]))[:3]
    x_axis = (H @ np.array([1,0,0,1]))[:3]
    y_axis = (H @ np.array([0,1,0,1]))[:3]
    z_axis = (H @ np.array([0,0,1,1]))[:3]

    ax.quiver(*origin, *(x_axis-origin) * length, color='r')
    ax.quiver(*origin, *(y_axis-origin) * length, color='g')
    ax.quiver(*origin, *(z_axis-origin) * length, color='b')

if __name__== "__main__":
    parser = argparse.ArgumentParser(description="Plot trajectory and coordinate transforms from all.json")
    parser.add_argument("trial_name", help="Trial name")
    parser.add_argument("--slam", action="store_true", help="Plot SLAM trajectory if available in all.json") 
    #'action' means if flag is present, automatically store true, if absent, store false
    parser.add_argument("--vicon", action="store_true", help="Plot Vicon trajectory if available in all.json")
    parser.add_argument("--stride", type=int, default=0, help="Stride to draw trajectory axes (default: 20)")
    parser.add_argument("--transforms_json", help="Optional transforms.json file", default=None)
    args = parser.parse_args()

    all_json_path = f"../out/{args.trial_name}_post/all.json"
    with open(all_json_path, 'r') as f:
        all_data = json.load(f)

    slam_poses = []
    vicon_poses = []

    for item in all_data:
        if item.get("type") == "slam_pose" and "T_body_world" in item:
            slam_poses.append(np.array(item["T_body_world"]))  # T_world_to_body
        if item.get("type") == "vicon_pose" and "T_body_world" in item:
            vicon_poses.append(np.array(item["T_body_world"]))  # T_world_to_body

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # --- SLAM trajectory ---
    if args.slam and slam_poses:
        positions_world = []
        for body_pose in slam_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='SLAM Trajectory', color='blue')
        ax.scatter(*positions_world[0], color='green', label='SLAM Start')
        ax.scatter(*positions_world[-1], color='red', label='SLAM End')

        if args.stride > 0:
            for i in range(0, len(slam_poses), args.stride):
                draw_axes(ax, slam_poses[i], length=0.4)

    # --- Vicon trajectory ---
    if args.vicon and vicon_poses:
        positions_world = []
        for body_pose in vicon_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='Vicon Trajectory', color='orange')
        ax.scatter(*positions_world[0], color='green', marker='^', label='Vicon Start')
        ax.scatter(*positions_world[-1], color='red', marker='^', label='Vicon End')

        if args.stride > 0:
            for i in range(0, len(vicon_poses), args.stride):
                draw_axes(ax, vicon_poses[i], length=0.4)

    # Common settings
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2,2)
    ax.set_zlim(-2, 2)
    ax.set_title("Trajectory and Static Coordinate Frames")
    ax.view_init(elev=20, azim=45)
    ax.legend()
    plt.show()
