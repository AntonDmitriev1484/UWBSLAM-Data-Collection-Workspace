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
    parser.add_argument("--vicon_tx", action="store_true", help="Plot Vicon TX Pose trajectory if available in all.json")    
    parser.add_argument("--uwbmap_vicon", action="store_true", help="Plot assisted UWB vicon body pose trajectory if available in all.json")    
    parser.add_argument("--stride", type=int, default=0, help="Stride to draw trajectory axes (default: 20)")
    parser.add_argument("--accel", action="store_true", help="Display accelerometer vectors rotated into world frame")
    parser.add_argument("--velocity", action="store_true", help="Display velocity vectors rotated into world frame")
    parser.add_argument("--transforms_json", help="Optional transforms.json file", default=None)
    args = parser.parse_args()

    all_json_path = f"../out/{args.trial_name}_post/all.json"
    with open(all_json_path, 'r') as f:
        all_data = json.load(f)

    transforms_path = f"../out/{args.trial_name}_post/transforms.json"
    with open(transforms_path, 'r') as f:
        Transforms = json.load(f)
    print(Transforms)
    T_imu_to_body = np.array(Transforms["T_imu_to_body"])

    slam_poses = []
    vicon_poses = []
    vicon_ts = []
    vicon_tx_poses = []
    uwbmap_vicon_poses = []

    accel_vectors = [] # accelertion in IMU frame
    accel_ts = []
    velocity_vectors = [] # Already aligned to vicon poses

    for item in all_data:
        if item.get("type") == "slam_pose" and "T_body_world" in item:
            slam_poses.append(np.array(item["T_body_world"]))  # T_world_to_body
        if item.get("type") == "vicon_pose" and "T_body_world" in item:
            vicon_poses.append(np.array(item["T_body_world"]))  # T_world_to_body
            velocity_vectors.append(np.array([item["v_world"]["vx"], item["v_world"]["vy"], item["v_world"]["vz"]]))
            vicon_ts.append(item["t"])
        if item.get("type") == "vicon_tx_pose" and "T_body_world" in item:
            vicon_tx_poses.append(np.array(item["T_body_world"]))  # T_world_to_body
        if item.get("type") == "assisted_uwb" and "T_body_world" in item:
            uwbmap_vicon_poses.append(np.array(item["T_body_world"]))  # T_world_to_body
            
        if item.get("type") == "imu":
            a_vector = np.array([item["ax"], item["ay"], item["az"]])
            accel_vectors.append(a_vector)  # T_world_to_body
            accel_ts.append(item["t"])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # --- SLAM trajectory ---
    if args.slam and slam_poses:
        positions_world = []
        for body_pose in slam_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='SLAM Body Trajectory', color='blue')
        ax.scatter(*positions_world[0], color='green', label='SLAM Start')
        ax.scatter(*positions_world[-1], color='red', label='SLAM End')

        if args.stride > 0:
            for i in range(0, len(slam_poses), args.stride):
                draw_axes(ax, slam_poses[i], length=0.4)

    # --- Vicon body frame trajectory ---
    if args.vicon and vicon_poses:
        positions_world = []
        imu_poses = []
        for body_pose in vicon_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
            imu_poses.append(np.linalg.inv(T_imu_to_body) @ body_pose)
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='Vicon Body Trajectory', color='green')
        ax.scatter(*positions_world[0], color='green', marker='^', label='Vicon Start')
        ax.scatter(*positions_world[-1], color='red', marker='^', label='Vicon End')

        if args.accel:

            skip = 100
            accel_ts = np.array(accel_ts)
            accel_vectors = np.array(accel_vectors)
            for i, (vpose, vts) in enumerate(zip(vicon_poses, vicon_ts)):
                if i % skip == 0:
                    # Find closest accelerometer measurement to pose
                    idx = np.argmin(np.abs(vts -accel_ts))
                    # Plot that vector in the world frame
                    accel_vector_imu_frame = accel_vectors[idx] / np.linalg.norm(accel_vectors[idx]) #unit vector
                    accel_vector_world_frame = T_imu_to_body[:3,:3] @ accel_vector_imu_frame # rotate vector into body frame

                    origin = np.linalg.inv(vpose)[:3,3]
                    ax.quiver(*origin, *accel_vector_world_frame, color='purple', length=0.3 )

        if args.velocity:
            skip = 100
            for i, (vpose, velocity_vector) in enumerate(zip(vicon_poses, velocity_vectors)):
                if i % skip == 0:
                    v = velocity_vector / 9
                    origin = np.linalg.inv(vpose)[:3,3]
                    ax.quiver(*origin, *v, color='pink', length=0.3 )

        if args.stride > 0:
            # for i in range (0, len(imu_poses), args.stride):
            #     draw_axes(ax, imu_poses[i], length=0.4)
            for i in range(0, len(vicon_poses), args.stride):
                draw_axes(ax, vicon_poses[i], length=0.4)

    # --- Vicon TX trajectory ---
    if args.vicon_tx and vicon_tx_poses:
        positions_world = []
        for body_pose in vicon_tx_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='Vicon TX Trajectory', color='green')
        ax.scatter(*positions_world[0], color='green', marker='^', label='Vicon Start')
        ax.scatter(*positions_world[-1], color='red', marker='^', label='Vicon End')

        if args.stride > 0:
            for i in range(0, len(vicon_tx_poses), args.stride):
                draw_axes(ax, vicon_tx_poses[i], length=0.4) 

    # --- Vicon TX trajectory ---
    if args.uwbmap_vicon and uwbmap_vicon_poses:
        positions_world = []
        for body_pose in uwbmap_vicon_poses:
            positions_world.append(np.linalg.inv(body_pose)[:3, 3])  # translation
        positions_world = np.array(positions_world)

        ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                label='Assisted UWB Trajectory', color='green')
        ax.scatter(*positions_world[0], color='green', marker='^', label='Vicon Start')
        ax.scatter(*positions_world[-1], color='red', marker='^', label='Vicon End')

        if args.stride > 0:
            for i in range(0, len(uwbmap_vicon_poses), args.stride):
                draw_axes(ax, uwbmap_vicon_poses[i], length=0.4) 

    # --- Anchor positions ---
    anchor_path = f"../out/{args.trial_name}_post/anchors.json"
    with open(anchor_path, 'r') as f:
        anchor_data = json.load(f)
        for d in anchor_data:
            ax.scatter(d["position"][0], d["position"][1], d["position"][2], color='purple')
            ax.text(
                d["position"][0],  # shift a bit in X
                d["position"][1],  # shift a bit in Y
                d["position"][2],
                d["ID"], color="black"
            )

    # --- Apriltag pose ---
    
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
