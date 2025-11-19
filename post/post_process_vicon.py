
from pathlib import Path
import pkgutil
import importlib
import inspect
import os
import json
import csv
import yaml
import argparse

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from types import SimpleNamespace

import shutil
import math
import copy

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *
from utils.vicon_utils import *


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Example usage:
# python3 post_process_vicon.py --trial_name irl3_los_walking --vicon_trial_name irl3_los_walking --map_vicon_to_uwb --no_orbslam True -c cam_target_daslab


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--vicon_available", action="store_true")
parser.add_argument("--slam_available", action="store_true") # If we have no orbslam data, just use vicon for everything instead.
# Since my code is a mess, I'm going to do this by just aliasing the vicon data into the slam arrays.

parser.add_argument("--calibration_file", "-c", type=str)
parser.add_argument("--crop_start", type=float, default=0) # Pass the relative timestamp from startthat you want to crop away all data before. Data will still be used to compute T.
parser.add_argument("--anchors_file", "-a", type=str)
parser.add_argument("--apriltags_file", "-p", type=str)

parser.add_argument("--synth_slam", type=float, nargs=2)

parser.add_argument("--interpolate_slam", "-i", default=0, type=int) # -i controls how many interpolated poses you want between each pair of SLAM poses.
parser.add_argument("--synth_uwb_f", default=0, type=int) # interpolate GT to this frequency, so that gtsam_test can use synthetic ranges.
parser.add_argument("--synth_slam_f", default=0, type=int) #  filter GT to this frequency, must be < 20 should really be named 'lower_slam_frequency'
parser.add_argument("--synth_vicon_f", default=0, type=int)

parser.add_argument("--leave_slam_frame", action="store_true")
parser.add_argument("--slam_f", default=0, type=float) # subsample the slam frequency

parser.add_argument("--map_vicon_to_uwb", action="store_true")
parser.add_argument("--include_vicon_tx_pose", action="store_true")
parser.add_argument("--vicon_for_worldframing", action="store_true")
# Instead of using AprilTag detection to convert SLAM to world frame, use Vicon.
# This lets us see a trajectory with just SLAM error, instead of SLAM + AprilTag error

args = parser.parse_args()

outpath = f'./out/{args.trial_name}_post'

out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_ml = f'{outpath}/ml'
out_synthetic = outpath+"/synthetic"
out_world = f'../world/{args.trial_name}' # Vicon can define apriltags and anchors set up in world frame

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_ml, exist_ok=True)
os.makedirs(out_synthetic, exist_ok=True)
os.makedirs(out_world, exist_ok=True)

in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_kalibr = f"../kalibr/camimu_out/{args.calibration_file}-camchain-imucam.yaml"
in_vicon = f"../vicon/out/{args.trial_name}.csv"
in_apriltags = f"../world/{args.apriltags_file}"
in_anchors = f"../world/{args.anchors_file}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

if args.vicon_available:
    vicon_data = parse_vicon_csv(in_vicon)

# headset_data contains the pose of the marker I had on the decawave antenna in the world frame.


# Need to maintain another array that we can buffer data to before dumping one sensor per csv
topic_to_processing = {
                '/uwb_ranges': (proc_range, []),
                  '/camera/camera/imu': (proc_imu, []),
                  '/camera/camera/infra1/image_rect_raw': (proc_infra1_frame, []),
                  '/camera/camera/infra2/image_rect_raw': (proc_infra2_frame, []),
}

all_data = []
dataset_topics = [ k for k,v in topic_to_processing.items()]
gt_standalone = []


rostypes = load_rostypes()
uwb_message_count = 0
processed_uwb_message = 0
# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=rostypes) as reader:
    connections = [x for x in reader.connections if x.topic in dataset_topics]
    for connection, timestamp, rawdata in reader.messages(connections=connections):

        try:
            msg = reader.deserialize(rawdata, connection.msgtype)
            proc, arr_ref = topic_to_processing[connection.topic]
            proc(msg, arr_ref)

        except Exception:
            print( "Exception! skipped message")
            continue  # optionally log here

# Processors functions have now buffered their individual topics into arr_ref
# This is useful for writing the same datastream to multiple files.
# Then, lastly, we can create all.json using the buffered measurements.


# # Filter for messages within bag timestamp range.
START = reader.start_time * 1e-9
END = reader.end_time * 1e-9
args.crop_start += START
print(f"ROS duration {START} - {END}")
print(f"Data start {START} cropped to {args.crop_start}")

vicon_name = f"Head{os.environ['USER_ID']}"
if os.environ['USER_ID'] == "2":
    if args.trial_name == "irl5_imu_bias_straight3" or args.trial_name == "irl5_imu_bias_worn":
        vicon_name = "Head4"
    elif args.trial_name =='irl4_free_together':
        vicon_name = "LeftRS2"

print(f"Using vicon {vicon_name}")

if args.vicon_available:
    vicon_data = crop_vicon(vicon_data, START, END)
    mobile_objects = [vicon_name]
    vicon_data = clean_vicon(vicon_data)

# Need to adjust vicon data to actual timestamps instead of just frame indices

def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


### Define all coordinate frames in T
T = SimpleNamespace()

# Transform from vicon marker on helmet, to center of RS camera (body frame)

# Vicon coordinate frames are marked with a 'v'

#Transform from vicon marker on anchor, to the center of the DW1000 UWB chip
T.T_vuwb_to_uwbtx = np.eye(4) # Probably better to express as a vector in the vUWB frame
T.T_vuwb_to_uwbtx[:3, 3] = [0.035, 0, 0] # 3cm down along x-axis.

T.T_vapril_to_world = np.eye(4)
if args.slam_available and not args.leave_slam_frame:
    #Transform from vicon marker to the center of an Apriltag
    # I manually selected the center of the apriltag to define the vicon frame
    T.T_vapril_to_world = slam_quat_to_HTM(vicon_data["April7"][0])

# The SLAM tracked body is the left camera.

# 'Head' refers to the vicon tracked head pose
T.T_imu_to_body = np.eye(4)
T.T_body_to_imu = np.linalg.inv(T.T_imu_to_body)

T_cam1_to_head = np.array([[-1 , 0, 0, 0.0175],
                           [0, 0, -1, -0.08],
                           [0, -1, 0, 0],
                           [0, 0, 0, 1]])
T.T_head_to_cam1 = np.linalg.inv(T_cam1_to_head)

with open(in_kalibr, 'r') as fs: calibration = yaml.safe_load(fs)
T.T_imu_to_cam1 = np.array(calibration['cam0']['T_cam_imu'])
T.T_cam1_to_body = T.T_imu_to_body @ np.linalg.inv(T.T_imu_to_cam1)
T.T_head_to_body = T.T_cam1_to_body @ T.T_head_to_cam1 # Seems to work better?


T.T_head_to_body = np.eye(4)
T.T_inertial_to_world = np.eye(4)

if 'irl5_imu_bias' in args.trial_name:
    print("Using irl5_imu_bias vicon2gt calibration")
    T_head_to_imu = np.array([
            [-0.998285,  0.0519228, -0.0270487, 0.000878626],
            [ 0.0292611, 0.0423338,  -0.998675, 0.0123712],
            [-0.0507089,  -0.997753, -0.0437805, -0.0607816],
            [ 0, 0, 0, 1]
    ])
    T.T_head_to_body = T_head_to_imu
    T.T_inertial_to_world[:3,:3] = np.array([           
        [1, -1.56931e-05, -0.000889414],
        [  0 ,    0.999844 ,  -0.0176416],
        [ 0.000889553 ,   0.0176416  ,   0.999844]]
        )
elif 'irl5_free2' == args.trial_name:
    #Use irl5_calibration2
    print("Use irl5_calibration2 vicon2gt results")
    T_head_to_imu = np.eye(4)
    T_head_to_imu[:3,:3] = np.array([  [  -0.996180546165 ,  0.0872390446955 ,-0.00369709652203],
                                        [0.0128724013916,    0.104847832127,   -0.994404964479],
                                        [-0.0863633065861 ,  -0.990654471135 ,  -0.105570346672]])
    T_head_to_imu[:3,3] = np.array(   [0.010305, -0.00379609, -0.0586384])
    T.T_head_to_body = T_head_to_imu

    T.T_inertial_to_world[:3,:3] = np.array([           
           [0.999105, 0.000610037,   0.0422917],
            [0 ,   0.999896 ,  -0.014423],
            [ -0.0422961 ,  0.0144101 ,   0.999001],]
    )

else: # For the real irl5 trails
    print("Using irl5 actual trials vicon2gt calibration")
    T_head_to_imu = np.eye(4)
    T_head_to_imu[:3,:3] = np.array([  [  -0.996180546165 ,  0.0872390446955 ,-0.00369709652203],
                                        [0.0128724013916,    0.104847832127,   -0.994404964479],
                                        [-0.0863633065861 ,  -0.990654471135 ,  -0.105570346672]])
    T_head_to_imu[:3,3] = np.array([   0.000878626 ,0.0123712, -0.0607816])

    T.T_head_to_body = T_head_to_imu

    T.T_inertial_to_world[:3,:3] = np.array([           
        [1, -1.56931e-05, -0.000889414],
        [  0 ,    0.999844 ,  -0.0176416],
        [ 0.000889553 ,   0.0176416  ,   0.999844]]
        )

T_decawave_to_head = np.eye(4)
T_decawave_to_head[:3,3] = np.array([-0.01, -0.0175, 0.0525])
T.T_head_to_decawave = np.linalg.inv(T_decawave_to_head)
T.T_body_to_decawave = T.T_head_to_decawave @ np.linalg.inv(T.T_head_to_body)

infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]

all_data_synthetic = [] # Keep interpolated points in a separate file from all.json

slam_json = []
if args.slam_available:
    slam_kf_data = np.loadtxt(in_slam_kf)
    slam_kf_data[:,0] *= 1e-9
    # Just temporarily
    # slam_data = np.loadtxt(in_slam)
    slam_data = np.loadtxt(in_slam_kf)
    print(f"{slam_data=}")
    slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'
    ZERO_TIMESTAMP = slam_data[0][0]

    # if not args.vicon_for_worldframing:
    #     # Use the apriltag to compute T.T_world_to_sorigin
    #     T = extract_apriltag_pose(slam_data, infra1_raw_frames, T, 
    #                                     in_kalibr, in_apriltags, 
    #                                     T_world_to_tag=np.linalg.inv(T.T_vapril_to_world))
    # else:
    #     # Find the closest Vicon pose to the SLAM origin
    #     cam1_slam =slam_data # Cam1 w.r.t SLAM origin
    #     cam1_vicon = np.array(vicon_data[vicon_name]) # Cam1 w.r.t vicon world origin

    #     # Pick the pose for alignment based of the best timestamp sync
    #     best_t = np.inf
    #     best_vicon_idx = 0
    #     best_slam_i = 0
    #     for i in range(cam1_slam.shape[0]):
    #         timediffs = np.abs( cam1_slam[i,0] - cam1_vicon[:,0])
    #         idx = np.argmin(timediffs) # 500 because we need to pick a point AFTER SLAM is initialized
    #         if best_t > timediffs[idx]:
    #             best_t = timediffs[idx]
    #             best_vicon_idx = idx
    #             best_slam_i = i
        
    #     print(f"{best_t=}")
    #     T.T_cam1_to_world = slam_quat_to_HTM(cam1_vicon[best_vicon_idx, :])
    #     T_cam1_to_sorigin = slam_quat_to_HTM(cam1_slam[best_slam_i, :])

    #     T.T_world_to_sorigin = T_cam1_to_sorigin @ np.linalg.inv(T.T_cam1_to_world)

    if args.leave_slam_frame:
        def slam_tracked_body_to_my_body(T_cam1_to_sorigin): # SLAM quat gives you the transform from cam1 frame to slam origin
            return T.T_cam1_to_body @ np.linalg.inv(T_cam1_to_sorigin)
        slam_body_poses, slam_body_velocities = aggregate_tracker(slam_tracked_body_to_my_body, slam_data)

        slam_freq = len(slam_data) / (END-args.crop_start) # Just for my specific test case at the moment
        skip = math.ceil(slam_freq / args.slam_f) # Number of vicon poses to skip in subsampling to synth slam frequency
        slam_body_poses = np.array(slam_body_poses)
        slam_body_poses = slam_body_poses[::skip] # Finally, subsample to required frequency

        # Convert from nparray to json format
        # Add Vicon poses to all_data
        slam_json = [ {
                "t": float(body_pose[0]),
                "type": "slam_pose",
                "T_body_world" : body_pose[1:].reshape((4,4)),
                "v_world": {
                        "vx": float(body_v[1]),
                        "vy": float(body_v[2]),
                        "vz": float(body_v[3])
                }
            } for body_pose, body_v in zip( list(slam_body_poses), list(slam_body_velocities))]

    else:
        def slam_tracked_body_to_my_body(T_cam1_to_sorigin): # SLAM quat gives you the transform from cam1 frame to slam origin
            return T.T_cam1_to_body @ np.linalg.inv(T_cam1_to_sorigin) @ T.T_world_to_sorigin
        slam_body_poses, slam_body_velocities = aggregate_tracker(slam_tracked_body_to_my_body, slam_data)

        slam_freq = len(slam_data) / (END-START)
        skip = math.ceil(slam_freq / args.slam_f) # Number of vicon poses to skip in subsampling to synth slam frequency
        slam_body_poses = np.array(slam_body_poses)
        slam_body_poses = slam_body_poses[::skip] # Finally, subsample to required frequency

            # Convert from nparray to json format
        # Add Vicon poses to all_data
        slam_json = [ {
                "t": float(body_pose[0]),
                "type": "slam_pose",
                "T_body_world" : body_pose[1:].reshape((4,4)),
                "v_world": {
                        "vx": float(body_v[1]),
                        "vy": float(body_v[2]),
                        "vz": float(body_v[3])
                }
            } for body_pose, body_v in zip( list(slam_body_poses), list(slam_body_velocities))]

### Write UWB data to its own csv file, and to all_data
uwb_csv = []
uwb_range_distribution = []
uwb_json = aggregate_uwb(topic_to_processing, uwb_csv, uwb_range_distribution)
with open(f'{out_ml}/uwb_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(uwb_csv))

### Write IMU data to its own csv file, and to all_data
imu_csv = []
imu_json = aggregate_imu(topic_to_processing, imu_csv)
with open(f'{out_ml}/imu_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(imu_csv))

# Compute accelerometer bias prior

imu = np.array(imu_csv)
accel_mag = np.linalg.norm(imu[:,1:4], axis=1)
window_size = 100
std_threshold = 0.07 # Select the threshold at which we determine motion to be higher than the base noise in the IMU
# This seems to work for realsense2
interval_end = 0 # Assume recording starts with nodes stationary
for i in range(0, imu.shape[0]-window_size):
    std_mag = np.std(accel_mag[i:i+window_size])
    if std_mag > std_threshold:
        interval_end = i
        break

# interval_end_tstp = imu[interval_end, 0]
interval_end_tstp = START+30
imu_stationary = imu[(START < imu[:,0]) & (imu[:,0] < interval_end_tstp)]
print(f"{imu[interval_end, 0]-START}s stationary IMU window detected from {START} - {imu[interval_end, 0]}")

accel_norm_tol = 0.1
for i in range(0, imu_stationary.shape[0]):
    if (accel_mag[i] > 9.81 + accel_norm_tol or accel_mag[i] < 9.81 - accel_norm_tol):
        print ( f" WARN: Bad box calibration, accel norm {accel_mag[i]} is far from 9.81")
        break
        # exit()

print(f" Average gravity magnitude in accelerometer frame {np.average(accel_mag[0:imu_stationary.shape[0]])}")

BIAS_STRATEGY = 'vicon'

mean_accel_imu = np.average(imu_stationary[:, 1:4], axis=0)
std_accel_imu = np.std(imu_stationary[:, 1:4], axis=0)
mean_gyro_imu = np.average(imu_stationary[:, 4:8], axis=0)
g_inertial = np.array([0,0,9.81])
R_inertial_to_imu = np.eye(4)
priors = {}
print(f"{mean_accel_imu=} {std_accel_imu=}")

if BIAS_STRATEGY == 'gram-schmidt':
    def gram_schmidt(gravity_vec):
        """Build orthonormal frame with z aligned to gravity_vec (IMU frame)."""
        z_axis = gravity_vec / np.linalg.norm(gravity_vec)
        v1 = np.array([1.0, 0.0, 0.0])
        v2 = np.array([0.0, 1.0, 0.0])

        x_axis = v1 - np.dot(v1, z_axis) * z_axis
        x_axis /= np.linalg.norm(x_axis)
        y_axis = v2 - np.dot(v2, z_axis) * z_axis - np.dot(v2, x_axis) * x_axis
        y_axis /= np.linalg.norm(y_axis)

        R_inertial_to_imu = np.column_stack((x_axis, y_axis, z_axis))
        # Gram-Schmidt gives us the rotation from the IMU to some (in this case Z-up) inertial frame
        # return np.linalg.inv(R_inertial_to_imu)
        return R_inertial_to_imu
    R_inertial_to_imu = gram_schmidt(mean_accel_imu)

    gyro_bias = mean_gyro_imu
    g_imu = R_inertial_to_imu @ g_inertial
    accel_bias = mean_accel_imu - g_imu # Assumes realsense is gravity aligned during calibration\
    print(f" {mean_accel_imu=} {g_imu=}")
    print(f"Gram Schmidt reported a: { accel_bias}, g: {gyro_bias}, R:{R_inertial_to_imu=} ")
    print(f"Ideal g_imu={[0,9.81, 0]} computed g_imu {g_imu} mag = {np.linalg.norm(g_imu)}")
    priors = {"accel_bias":accel_bias, "gyro_bias":gyro_bias, "velocity":np.array([0,0,0]), "t_end_calibration":interval_end_tstp}

    print(f"Quick average reports a: { mean_accel_imu + np.array([0,9.81,0])} g: {gyro_bias}")

if BIAS_STRATEGY == 'average':

    R_inertial_to_imu = np.array([[1,0,0],[0,0,-1],[0,1,0]])

    gyro_bias = mean_gyro_imu
    g_imu = R_inertial_to_imu @ g_inertial
    accel_bias = mean_accel_imu - g_imu # Assumes realsense is gravity aligned during calibration\
    print(f" {mean_accel_imu=} {g_imu=}")
    print(f"Average reported a: { accel_bias}, g: {gyro_bias}, R:{R_inertial_to_imu=} ")
    print(f"Ideal g_imu={[0,9.81, 0]} computed g_imu {g_imu} mag = {np.linalg.norm(g_imu)}")
    priors = {"accel_bias":accel_bias, "gyro_bias":gyro_bias, "velocity":np.array([0,0,0]), "t_end_calibration":interval_end_tstp}

    print(f"Quick average reports a: { mean_accel_imu + np.array([0,9.81,0])} g: {gyro_bias}")
### Apply T to the Vicon tracking data of cam1

synth_vicon_json = []
vicon_json = []
if args.vicon_available:
    def vicon_tracked_body_to_my_body(T_head_to_world):
        # By default, vicon pose tracking gives you the T_head_to_world
        # return T.T_cam1_to_body @ T.T_head_to_cam1 @ np.linalg.inv(T_head_to_world) #output world to body
        return T.T_head_to_body @ np.linalg.inv(T_head_to_world)

    vicon_body_poses, vicon_body_velocities = aggregate_tracker(vicon_tracked_body_to_my_body, np.array(vicon_data[vicon_name]))

    # Convert from nparray to json format
    # Add Vicon poses to all_data
    vicon_json = [ {
            "t": float(body_pose[0]),
            "type": "vicon_pose",
            "T_body_world" : body_pose[1:].reshape((4,4)),
            "v_world": {
                    "vx": float(body_v[1]),
                    "vy": float(body_v[2]),
                    "vz": float(body_v[3])
            }
        } for body_pose, body_v in zip( list(vicon_body_poses), list(vicon_body_velocities))]
    
    if BIAS_STRATEGY == 'vicon':
        vicon_stationary =  vicon_body_poses[(START < vicon_body_poses[:,0]) & (vicon_body_poses[:,0] < interval_end_tstp)]
        T_world_to_body = vicon_stationary[0, 1:].reshape((4,4))
        
        R_world_to_imu = T_world_to_body[:3,:3]
        R_inertial_to_world = T.T_inertial_to_world[:3,:3]
        gyro_bias = mean_gyro_imu
        R_inertial_to_imu = R_world_to_imu @ R_inertial_to_world
        g_imu = R_inertial_to_imu @ g_inertial
        accel_bias = mean_accel_imu - g_imu # Assumes realsense is gravity aligned during calibration\
        print(f"Vicon reported: \nba: { accel_bias} \nbg: {gyro_bias} \nR:{R_inertial_to_imu=} ")
        print(f" Mean a_imu={mean_accel_imu}")
        print(f"Ideal g_imu={[0,9.81, 0]} computed g_imu {g_imu} mag = {np.linalg.norm(g_imu)}")
        priors = {"accel_bias":accel_bias, "gyro_bias":gyro_bias, "velocity":np.array([0,0,0]), "t_end_calibration":interval_end_tstp}


synth_slam_json = []
if args.synth_slam:
    print(args.synth_slam)

    vicon_freq = len(vicon_data[vicon_name]) / (END-START)
    skip = math.ceil(vicon_freq / args.synth_slam[0]) # Number of vicon poses to skip in subsampling to synth slam frequency

    JUST_SUBSAMPLE = True
    max_t_err = 0
    max_R_err = 0
    t_rw_step = 0
    R_rw_step = 0

    # Define error random walk
    # max_t_err = 0.15 # Allowing for at most 15cm trans error
    # max_R_err = 5 # Allowing for at most 5 deg rot error

    # t_rw_step = 0.005 # How much error change do we want to see per vicon pose? # 1mm change is at most 10cm drift / s
    # R_rw_step = 0.01
    t_err = np.zeros(3)
    R_err = np.eye(3)

    rng = np.random.default_rng(0)

    tracker_data_tum = np.array(vicon_data[vicon_name])
    slam_body_poses = [] # Synthetic slam body poses

    if not JUST_SUBSAMPLE:
        # First generate all of the error vectors
        t_err_ = []
        R_err_ = []
        for i in range(tracker_data_tum.shape[0]-1):

            # Take the random walk step in our error. 
            # Clamp t_err and R_err to bounds after step.
            delta_t = rng.normal(0, t_rw_step, 3) # Generate step from Gauss
            # We want 0 preturbation along the local z axis. I.e. no forward drift. But the Z-axis will not always be forward!
            new_t_err = t_err + delta_t # Apply step
            if np.linalg.norm(new_t_err) <= max_t_err: # Clamp
                t_err = new_t_err

            # print(f"{t_err=} {delta_t=}")

            delta_rvec = rng.normal(0, np.deg2rad(R_rw_step), 3)  # Generate step from Gauss
            delta_r = R.from_rotvec(delta_rvec).as_matrix()
            new_R_err = delta_r @ R_err # Apply step
            angle = R.from_matrix(new_R_err).magnitude()
            if angle <= np.deg2rad(max_R_err): # Clamp
                R_err = new_R_err

            t_err_.append(t_err)
            R_err_.append(R_err)

        # Smooth over translation error
        t_err_ = np.array(t_err_)
        R_err_ = np.array(R_err_)


        print(f"{t_err_.shape=} {R_err_.shape=}")

        window = 25
        cumsum = np.cumsum(t_err_, axis = 0) 
        t_err_ = (cumsum[window:] - cumsum[:-window]) / float(window)
        R_err_ = R_err_[window:]


        print(f"{t_err_.shape=} {R_err_.shape=}")

        # Apply error vectors
        for j in range(t_err_.shape[0]):

            # Pose of our errored frame, in the current tracker frame. -> Error is applied in the tracker frame.
            # First translate by t_err, then rotate by R_err
            # this applies the translation error vector in the tracker coord frame.

            t_err_transform = np.eye(4)
            t_err_transform[:3,3] = t_err_[j]
            R_err_transform = np.eye(4)
            R_err_transform[:3,:3] = R_err_[j]
            error_transform = R_err_transform @ t_err_transform

            tracker_pose = slam_quat_to_HTM(tracker_data_tum[j, :])   # tracker pose as HTM

            # T_world_to_slam = T_world_to_error
            slam_body_pose =  np.linalg.inv(error_transform) @ np.linalg.inv(tracker_pose) #  T_tracker_to_error @ T_world_to_tracker
            slam_body_poses.append([tracker_data_tum[j, 0]] + list(slam_body_pose.flatten()))

        slam_body_poses = np.array(slam_body_poses)
        slam_body_poses = slam_body_poses[::skip] # Finally, subsample to required frequency

    else:# For simplicity in testing irl5 imu. You can just subsample the vicon poses as synthetic SLAM
        slam_body_poses = vicon_body_poses[::skip]

    # Add Vicon poses to all_data

    # synth slam is being generated with all the same timestamps
    synth_slam_json = [ {
            "t": float(body_pose[0]),
            "type": "synth_slam_pose",
            "T_body_world" : body_pose[1:].reshape((4,4)),
        } for body_pose in list(slam_body_poses)]



# If we're using real UWB ranges, but have no compass
# We interpolate on SLAM poses to match a synthetic orientation to that UWB range
assisted_uwb_json = []
if args.map_vicon_to_uwb:
    assisted_uwb_json = aggregate_assisted_uwb(uwb_json, vicon_tracked_body_to_my_body, np.array(vicon_data[vicon_name]), 100)

vicon_uwbtx_json = []
if args.include_vicon_tx_pose:
    # Has problems because in the IRL datasets we frequently lose tracking of the UWB1
    def vicon_tracked_uwb1_to_uwb1_tx(T_vuwb1_to_world): 
        return T.T_vuwb_to_uwbtx @ np.linalg.inv(T_vuwb1_to_world)
    vicon_tx_poses, _ = aggregate_tracker(vicon_tracked_uwb1_to_uwb1_tx, np.array(vicon_data["UWB1"]))

    vicon_uwbtx_json = [ {
        "t": float(body_pose[0]),
        "type": "vicon_tx_pose",
        "T_body_world" : body_pose[1:].reshape((4,4)),
    } for body_pose in list(vicon_tx_poses)]

### Write Infra1 frames to output directory, and provide references in all_data
infra1_json = aggregate_infra1(topic_to_processing, out_infra1)

### Write Infra2 frames to output directory, and provide references in all_data
infra2_json = aggregate_infra2(topic_to_processing, out_infra2)

    # Compose the final factor graph dataset
all_data = uwb_json + imu_json + infra1_json + infra2_json + vicon_json + slam_json + assisted_uwb_json + vicon_uwbtx_json + synth_slam_json

# TODO: Compose the final synthetic dataset

# TODO: Weird error, I will come back to this later.
# vicon_body_poses_tum = [ slam_HTM_to_TUM(pose) for pose in vicon_body_poses]
# with open(f'{out_ml}/vbody_poses_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(vicon_body_poses))
# with open(f'{outpath}/vbody_poses_world_frame_tum.txt', 'w') as fs: csv.writer(fs, delimiter=' ').writerows(filtt2(vicon_body_poses_tum))

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, '__dict__'):
            return vars(obj)
        return super().default(obj)

### Copy all world information: T, anchors, apriltags, to output

if args.vicon_available:
    # Use Vicon information to compute the world frame for our tags and anchors
    world_frame_anchors = []
    world_frame_tags = {}
    for tracked_name, data in vicon_data.items():
        if "UWB" in tracked_name and tracked_name not in mobile_objects:
            # Compute the tx point over all poses, then average them.
            uwb_tx_position = get_tx_position(T.T_vuwb_to_uwbtx, data)
            world_frame_anchors.append({
                "ID": tracked_name.replace("UWB", ""),
                "position": uwb_tx_position
            })
        if "April" in tracked_name:
            # Just dump the first transform for that tag in
            world_frame_tags[tracked_name.replace("April","")] = slam_quat_to_HTM(data[0])[1:]

    out_anchors = open(f'{out_world}/anchors_{args.trial_name}.json', 'w')
    json.dump(world_frame_anchors, out_anchors, cls=NumpyEncoder, indent=1)
    out_anchors_trial = open(f'{outpath}/anchors.json', 'w')
    json.dump(world_frame_anchors, out_anchors_trial, cls=NumpyEncoder, indent=1)

    out_tags = open(f'{out_world}/apriltags_{args.trial_name}.json', 'w')
    json.dump(world_frame_tags, out_tags, cls=NumpyEncoder, indent=1)
    out_tags_trial = open(f'{outpath}/apriltags.json', 'w')
    json.dump(world_frame_tags, out_tags_trial, cls=NumpyEncoder, indent=1)


with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(T), fs, cls=NumpyEncoder, indent=1)

# Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
if args.vicon_available:
    print("Checking frequency of real data")
    print(f" Measured UWB frequency {uwb_message_count / (END-START)}")
    print(f" Measured vicon frequency {len(vicon_data[vicon_name]) / (END-START)}")
if args.synth_slam:
    print("Checking frequency of synthetic SLAM")
    print(f" Measured Synth SLAM frequency {len(synth_slam_json) / (END-START)}")


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])

calibration_data = [a for a in all_data if a["t"] < interval_end_tstp]
main_data = [a for a in all_data if a["t"] > interval_end_tstp]
json.dump(calibration_data, open(outpath+"/calibration.json", 'w'), cls=NumpyEncoder, indent=1)
json.dump(main_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)
json.dump(priors, open(outpath+"/priors.json", 'w'), cls=NumpyEncoder, indent=1)

# All data synthetic is (all real data except slam) + (real SLAM (filtered) + synthetic UWB (created from interpolating on real SLAM))
all_data_synthetic = filtt( [a for a in all_data if not a["type"] == "slam_pose"] + all_data_synthetic) 
all_data_synthetic = sorted(all_data_synthetic, key=lambda x: x["t"])

json.dump(all_data_synthetic, open(outpath+"/synthetic"+f"/all_synthetic_{args.synth_slam_f}_{args.synth_uwb_f}.json", 'w'), cls=NumpyEncoder, indent=1)
# So all synthetic files will have a unique name


json.dump(args.__dict__, open(out_synthetic+f"/all_synthetic_{args.synth_slam_f}_{args.synth_uwb_f}_meta.json", 'w'), cls=NumpyEncoder, indent=1)
