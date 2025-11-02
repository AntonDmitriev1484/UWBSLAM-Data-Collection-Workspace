#!/usr/bin/env python3
import argparse
import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import rosbag2_py
from rosbag2_py import TopicMetadata
import shutil
from rclpy.serialization import serialize_message
import sys
from pathlib import Path
# Add ../post/utils to sys.path

def HTM_to_TUM(T): # 2D pose matrix to TUM format
    # Extract translation and rotation
    t = T[:3, 3]
    R_mat = T[:3, :3]
    quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

    # Non timestamped
    return [ t[0], t[1], t[2], quat[0], quat[1], quat[2], quat[3]]
def slam_quat_to_HTM(nparr): # Doesnt timestamp
    translation = nparr[1:4]
    quat = nparr[4:8]
    
    r = R.from_quat(quat)
    rotation_matrix = r.as_matrix()

    # Assemble homogeneous transformation matrix (4x4)
    H = np.eye(4)
    H[:3, :3] = rotation_matrix
    H[:3, 3] = translation

    return H
# Expects data to be input as an HTM
# Pose can be passed in any frame, but be mindful of the SLERP left hand coordinate system problem
# Pass a target_timestamp, first < target < second
def interpolate_pose(first_pose, first_timestamp, second_pose, second_timestamp, target_timestamp, n_points):

    # Now interpolate between these two poses
    interp_interval = [first_timestamp, second_timestamp]
    interp_timestamps = np.linspace(first_timestamp, second_timestamp, n_points)

    # Use Slerp to interpolate on SO(3) rotations
    interp_rots = R.from_matrix([first_pose[:3, :3], second_pose[:3, :3]])
    slurpy = Slerp(interp_interval, interp_rots)
    interpolated_rotations = slurpy(interp_timestamps)

    # Use linspace to interpolate on R3 positions
    interpolated_positions = np.linspace(first_pose[:3, 3], second_pose[:3, 3], n_points)

    # Fetch the closest interpolation timestamp to the uwb measurement, and map that interpolated pose to the measurement
    idx_match = np.argmin(np.abs(interp_timestamps - target_timestamp))

    interp_pose = np.eye(4)
    interp_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
    interp_pose[:3, 3] = interpolated_positions[idx_match]

    return interp_pose

def clean_vicon(vicon_data):

    # If you're mobile and translation suddenly drop to 0, that means tracking was lost. interpolate that thang

    data = vicon_data
    # In case we start off at a 0 pose, find the first non-zero pose
    # and set that to be our start pose

    def is_outlier(tum_pose):
        norm = np.linalg.norm(np.array(tum_pose)[1:])
        return norm <= 1e-5

    # If our starting pose is an outlier and we have nothing to interpolate between
    start_pose = None
    for i in range(0, len(data)):
        if not is_outlier(data[i]): 
            start_pose = np.array(data[i]) # Next valid TUM timestamped pose
            break

    for p in range(0,i):
        data[p] = start_pose

    # Now clean
    for i in range(1, len(data)):
        if is_outlier(data[i]):

            last_pose = np.array(data[i-1]) # Last valid TUM timestamped pose
            next_pose = None
            interp_pose = None
            for j in range(i+1, len(data)): # Find next valid TUM timestamped pose
                # print(data)
                if not is_outlier(data[j]): 
                    next_pose = np.array(data[j]) # Next valid TUM timestamped pose
                    current_timestamp = data[i][0]
                    interp_pose = interpolate_pose(
                        slam_quat_to_HTM(last_pose), last_pose[0],
                        slam_quat_to_HTM(next_pose), next_pose[0],
                        current_timestamp, 100
                    )
                    break

            if interp_pose is not None:
                interp_pose = HTM_to_TUM(interp_pose) # Returns a non timestamped HTM
                data[i] = np.insert(interp_pose, 0, current_timestamp) #I'm pretty sure this mutates the original array?
    for i in range(1, len(data)):
        last_pose = np.array(data[i-1]) # Last valid TUM timestamped pose
        this_pose = np.array(data[i])
        last_rot = R.from_quat(last_pose[4:8])
        this_rot = R.from_quat(this_pose[4:8])

        # print(f"{last_rot.as_rotvec()=} {this_rot.as_rotvec()=}")

        angle_diff_deg = np.degrees(np.linalg.norm((this_rot * last_rot.inv()).as_rotvec()))
        if angle_diff_deg > 145:
            corrected_pose = last_pose.copy()
            corrected_pose[0:4] = this_pose[0:4] # New position and timestamp but the last (correct) rotation.
            data[i] = corrected_pose
    return data

def tum_to_htm(tum_row):
    _, x, y, z, qx, qy, qz, qw = tum_row
    R_mat = R.from_quat([qx, qy, qz, qw]).as_matrix()
    HTM = np.eye(4)
    HTM[:3, :3] = R_mat
    HTM[:3, 3] = [x, y, z]
    return HTM


def load_vicon_poses(csv_path, target):
    poses = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = next(reader, None)  # skip header
        for row in reader:
            if len(row) < 11:
                continue
            if row[2] == target and row[3] == target:
                try:
                    t = float(row[0]) / 1e3  # convert from ms to s
                    x, y, z = map(float, row[4:7])
                    x /= 1000
                    y/= 1000
                    z/= 1000 # convert to m scale
                    qx, qy, qz, qw = map(float, row[7:11])
                    poses.append([t, x, y, z, qx, qy, qz, qw])
                except ValueError:
                    continue
    return poses


def create_pose_msg(t, x, y, z, qx, qy, qz, qw):
    msg = PoseStamped()
    msg.header.frame_id = "vicon"
    msg.header.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def get_imu_time_bounds(imu_bag_path):
    """Read IMU topic timestamps and return (start_time, end_time) in seconds."""
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=imu_bag_path, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)

    start_t, end_t = None, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/camera/camera/imu":
            if start_t is None:
                start_t = t
            end_t = t
    if start_t is None or end_t is None:
        raise RuntimeError("No IMU messages found in bag.")
    return start_t / 1e9, end_t / 1e9  # convert from ns → s


def write_ros2_bag(trial_name, vicon_poses, imu_bag_path):
    output_bag = f"./vicon2gt/in/{trial_name}"

    storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    OUT_VICON_TOPIC = '/vicon/firefly_sbx/firefly_sbx'
    OUT_IMU_TOPIC = '/imu0'

    writer.create_topic(
        TopicMetadata(
            name=OUT_VICON_TOPIC,
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
    )
    writer.create_topic(
        TopicMetadata(
            name=OUT_IMU_TOPIC,
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr'
        )
    )

    # --- Crop Vicon poses to IMU time range ---
    start_t, end_t = get_imu_time_bounds(imu_bag_path)
    print(f"[INFO] Cropping Vicon data to IMU time range: {start_t:.3f} → {end_t:.3f}")
    vicon_poses = [p for p in vicon_poses if start_t <= p[0] <= end_t]
    print(f"[INFO] After cropping: {len(vicon_poses)} Vicon poses remain")

    print(f"Priors for vicon2gt launchfile:")
    R_head_to_world = R.from_quat(vicon_poses[0][4:8]).as_matrix()
    t_world_to_head = vicon_poses[0][1:4]
    print(f"R_BtoI {list(R_head_to_world.flatten())}")
    print(f"p_BinI {t_world_to_head}")

    # --- Write Vicon poses ---
    for tum_row in vicon_poses:
        t, x, y, z, qx, qy, qz, qw = tum_row
        pose_msg = create_pose_msg(t, x, y, z, qx, qy, qz, qw)
        serialized = serialize_message(pose_msg)
        writer.write(OUT_VICON_TOPIC, serialized, int(t * 1e9))

    # --- Copy IMU data ---
    reader = rosbag2_py.SequentialReader()
    imu_storage = rosbag2_py.StorageOptions(uri=imu_bag_path, storage_id="sqlite3")
    reader.open(imu_storage, converter_options)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/camera/camera/imu":
            writer.write(OUT_IMU_TOPIC, data, t)

    print(f"✅ ROS2 bag written to: {output_bag}")
    return output_bag


def convert_ros2_to_ros1(ros2_bag_path, trial_name):
    ros1_bag_path = f"./vicon2gt/in/{trial_name}.bag"
    # os.system(f"ros2 bag convert --output {ros1_bag_path} --input {ros2_bag_path}")
    os.system(f'rosbags-convert --src \"{ros2_bag_path}\" --dst \"{ros1_bag_path}\"')
    print(f"✅ Converted to ROS1 bag: {ros1_bag_path}")
    return ros1_bag_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Vicon CSV + IMU ROS2 bag into a combined ROS1 bag.")
    parser.add_argument("-t", "--trial_name", required=True, help="Trial name (e.g. 'trial_001')")
    args = parser.parse_args()

    trial_name = args.trial_name
    vicon_csv = f"./out/{trial_name}.csv"
    imu_bag = f"../collect/ros2/{trial_name}"

    # --- Cleanup old data ---
    ros2_bag_dir = f"./vicon2gt/in/{trial_name}/"
    ros1_bag_file = f"./vicon2gt/in/{trial_name}.bag"
    if os.path.exists(ros2_bag_dir):
        print(f"[INFO] Removing existing ROS2 bag directory: {ros2_bag_dir}")
        shutil.rmtree(ros2_bag_dir)
    if os.path.exists(ros1_bag_file):
        print(f"[INFO] Removing existing ROS1 bag file: {ros1_bag_file}")
        os.remove(ros1_bag_file)

    if not os.path.exists(vicon_csv):
        raise FileNotFoundError(f"Missing Vicon CSV: {vicon_csv}")
    if not os.path.exists(imu_bag):
        raise FileNotFoundError(f"Missing ROS2 bag: {imu_bag}")

    TARGET = "Head4"
    vicon_poses = load_vicon_poses(vicon_csv, TARGET)
    vicon_poses = clean_vicon(vicon_poses) # Including cleaning code with copy paste the good ol fashioned way
    print(f"Loaded {len(vicon_poses)} poses from {vicon_csv}")

    ros2_bag_path = write_ros2_bag(trial_name, vicon_poses, imu_bag)
    convert_ros2_to_ros1(ros2_bag_path, trial_name)
