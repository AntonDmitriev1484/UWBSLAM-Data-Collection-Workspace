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

def main():
    all_json_path = "/home/antond2/ws/post/out/winerf_orbslam_test4_post/all.json"  # Hardcode your file path
    stride = 100  # Stride for drawing axes
    Z_fixed = 0.275  # Circle height
    circle_radius = 0.5
    circle_center = np.array([0, 3, Z_fixed])

    # Load measured poses
    with open(all_json_path, 'r') as f:
        all_data = json.load(f)

    transforms = []
    positions = []

    for item in all_data:
        if item.get("type") == "slam_pose" and "T_body_world" in item:
            T = np.array(item["T_body_world"])
            transforms.append(T)
            positions.append(T[:3, 3])

    positions = np.array(positions)
    N = len(positions)

    # Generate GT poses along perfect circle
    theta = np.linspace(0, 2 * np.pi, N, endpoint=False)
    x_gt = circle_radius * np.cos(theta) + circle_center[0]
    y_gt = circle_radius * np.sin(theta) + circle_center[1]
    z_gt = np.full(N, circle_center[2])
    gt_positions = np.vstack((x_gt, y_gt, z_gt)).T

       # For each measured point, find closest GT point (reuse allowed)
       # Points are in the wrong order, so instead of sorting and doign the kinematics I'm just going to run the comparison
       # based on the closest point.
    matched_gt_positions = np.zeros_like(positions)

    for i, pos in enumerate(positions):
        dists = np.linalg.norm(gt_positions - pos, axis=1)
        min_idx = np.argmin(dists)
        matched_gt_positions[i] = gt_positions[min_idx]

    # Compute error
    errors = np.linalg.norm(positions - matched_gt_positions, axis=1)
    mean_err = np.mean(errors)
    rmse = np.sqrt(np.mean(errors**2))

    print(f"Mean position error vs closest GT point: {mean_err:.4f} m")
    print(f"RMSE: {rmse:.4f} m")


    # # Compute error
    # errors = np.linalg.norm(positions - matched_gt_positions, axis=1)
    # mean_err = np.mean(errors)
    # rmse = np.sqrt(np.mean(errors**2))

    # print(f"Mean position error vs best-matching GT points: {mean_err:.4f} m")
    # print(f"RMSE: {rmse:.4f} m")

    # Compute mean error
    errors = np.linalg.norm(positions - gt_positions, axis=1)
    mean_err = np.mean(errors)
    # rmse = np.sqrt(np.mean(np.sum(errors**2)))
    print(f"Mean position error vs perfect circle: {mean_err:.4f} meters")

    # Plot measured trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(0, N, 25):
        ax.quiver(
            matched_gt_positions[i, 0], matched_gt_positions[i, 1], matched_gt_positions[i, 2],
            positions[i, 0] - matched_gt_positions[i, 0],
            positions[i, 1] - matched_gt_positions[i, 1],
            positions[i, 2] - matched_gt_positions[i, 2],
            color='orange',
            arrow_length_ratio=0.1,
            linewidth=1
        )

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Measured Trajectory', color='blue')
    ax.scatter(*positions[0], color='green', label='Start')
    ax.scatter(*positions[-1], color='red', label='End')

    # Plot GT circle
    ax.plot(x_gt, y_gt, z_gt, color='magenta', label=f'GT Circle at Z={Z_fixed}')

    if stride > 0:
        for i in range(0, len(transforms), stride):
            draw_axes(ax, transforms[i], length=0.4)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Trajectory vs GT Circle")
    ax.view_init(elev=20, azim=45)
    set_axes_equal(ax)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
