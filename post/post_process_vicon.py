
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
parser.add_argument("--slam_f", default=0, type=float)

parser.add_argument("--calibration_file", "-c", type=str)
parser.add_argument("--crop_start", default = 0, type=float) # Pass the ROS timestamp that you want to crop away all data before. Data will still be used to compute transforms.
parser.add_argument("--anchors_file", "-a", type=str)
parser.add_argument("--apriltags_file", "-p", type=str)

parser.add_argument("--interpolate_slam", "-i", default=0, type=int) # -i controls how many interpolated poses you want between each pair of SLAM poses.
parser.add_argument("--synth_uwb_f", default=0, type=int) # interpolate GT to this frequency, so that gtsam_test can use synthetic ranges.
parser.add_argument("--synth_slam_f", default=0, type=int) #  filter GT to this frequency, must be < 20 should really be named 'lower_slam_frequency'
parser.add_argument("--synth_vicon_f", default=0, type=int)

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

vicon_data = parse_vicon_csv(in_vicon) # TODO: Write a parsing function for the vicon files

# headset_data contains the pose of the marker I had on the decawave antenna in the world frame.

# SOURCE = int(os.environ.get("USER_ID"))
SOURCE = 1 # for IRL3 trials, although they were collected on NUC2, the decawave had ID 1

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
print(rostypes)

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
            if connection.msgtype == "beluga_messages/msg/BelugaRanges": 
                processed_uwb_message +=1
                uwb_message_count += 1

        except Exception:
            print( "skipped UWB message")
            if connection.msgtype == "beluga_messages/msg/BelugaRanges": 
                uwb_message_count +=1
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

vicon_data = crop_vicon(vicon_data, START, END)

mobile_objects = ["LeftRS", "UWB1"]
vicon_data = clean_vicon(vicon_data)

# Need to adjust vicon data to actual timestamps instead of just frame indices

def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


### Define all coordinate transforms
T = SimpleNamespace()


### IRL3 Rig and Vicon Transforms

#Transform from vicon marker on anchor, to the center of the DW1000 UWB chip
T.T_vuwb_to_uwbtx = np.eye(4) # Probably better to express as a vector in the vUWB frame
# T.T_vuwb_to_uwbtx[:3, 3] = [0.035, 0, 0] # 3cm down along x-axis. TODO: Is the signage here correct? No
T.T_vuwb_to_uwbtx[:3, 3] = [-0.035, 0, 0]
# This should be [R_vuwb_to_uwbtx | t_uwb_tx_to_vuwb]

#Transform from vicon marker to the center of an Apriltag
# I manually selected the center of the apriltag to define the vicon frame
T.T_vapril_to_world = slam_quat_to_HTM(vicon_data["April7"][0])

T.T_vcam1_to_cam1 = np.eye(4) # This trial tracked cam1 with the markers on the camera (roughly)

T.T_cam1_to_uwb = np.eye(4) # T_cam1_to_uwb = [R_cam1_to_uwb | t_uwb_to_cam1]
T.T_cam1_to_uwb[:3, 3] = np.array([0.0725, 0.03, 0.061])

# Transforms I'm defining

T.T_imu_to_body = np.eye(4) # IMU is the body frame, like I defined for the IRL5 trials
T.T_body_to_imu = np.linalg.inv(T.T_imu_to_body)

with open(in_kalibr, 'r') as fs: calibration = yaml.safe_load(fs)
T.T_imu_to_cam1 = np.array(calibration['cam0']['T_cam_imu'])
T.T_cam1_to_body = np.linalg.inv(T.T_imu_to_cam1)

T.T_body_to_decawave = T.T_cam1_to_uwb @ T.T_imu_to_cam1

# Since SLAM frame is assumed to be gravity aligned, and we're mainly using this for SLAM
T.T_inertial_to_world = np.eye(4)

# Transforms.T_body_to_decawave[:3,3] = np.array([-0.045, -0.15, -0.025]) # For uwb_calibration_trans
# Transforms.T_body_to_decawave[:3,3] = np.array([-0.12, 0.015, -0.1])


infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]

all_data_synthetic = [] # Keep interpolated points in a separate file from all.json

slam_json = []
if args.slam_available:
    slam_kf_data = np.loadtxt(in_slam_kf)
    slam_kf_data[:,0] *= 1e-9
    slam_data = np.loadtxt(in_slam)
    slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'
    ZERO_TIMESTAMP = slam_data[0][0]

    def slam_tracked_body_to_my_body(T_cam1_to_sorigin):
        return T.T_cam1_to_body @ np.linalg.inv(T_cam1_to_sorigin)
    # Our output body poses will be the IMU in the SLAM frame.

    slam_body_poses, slam_body_velocities = aggregate_tracker(slam_tracked_body_to_my_body, slam_data)

    if args.slam_f != 0:
        slam_freq = len(slam_data) / (END-args.crop_start) # Just for my specific test case at the moment
        skip = math.ceil(slam_freq / args.slam_f) # Number of vicon poses to skip in subsampling to synth slam frequency
        slam_body_poses = np.array(slam_body_poses)
        slam_body_poses = slam_body_poses[::skip] # Finally, subsample to required frequency
    
        # Convert from nparray to json format
    # Add Vicon poses to all_data
    slam_json = [ {
            "t": float(body_pose[0]),
            "type": "slam_pose",
            "src": SOURCE,
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

### Apply transforms to the Vicon tracking data of cam1
# Mostly ignoring this on this branch
synth_vicon_json = []
vicon_json = []
if args.vicon_available:
    def vicon_tracked_body_to_my_body(T_vcam1_to_world):
        # By default, vicon pose tracking gives you the T_vcam1_to_world
        return T.T_cam1_to_body @ np.linalg.inv(T_vcam1_to_world)
    vicon_body_poses, vicon_body_velocities = aggregate_tracker(vicon_tracked_body_to_my_body, np.array(vicon_data["LeftRS"]))

    # Convert from nparray to json format
    # Add Vicon poses to all_data
    vicon_json = [ {
            "t": float(body_pose[0]),
            "type": "vicon_pose",
            "src": SOURCE,
            "T_body_world" : body_pose[1:].reshape((4,4)),
            "v_world": {
                    "vx": float(body_v[1]),
                    "vy": float(body_v[2]),
                    "vz": float(body_v[3])
            }
        } for body_pose, body_v in zip( list(vicon_body_poses), list(vicon_body_velocities))]

# If we're using real UWB ranges, but have no compass
# We interpolate on SLAM poses to match a synthetic orientation to that UWB range
assisted_uwb_json = []
if args.map_vicon_to_uwb:
    assisted_uwb_json = aggregate_assisted_uwb(uwb_json, vicon_tracked_body_to_my_body, np.array(vicon_data["LeftRS"]), 100)

vicon_uwbtx_json = []
if args.include_vicon_tx_pose:
    # Has problems because in the IRL datasets we frequently lose tracking of the UWB1
    def vicon_tracked_uwb1_to_uwb1_tx(T_vuwb1_to_world): 
        return T.T_vuwb_to_uwbtx @ np.linalg.inv(T_vuwb1_to_world)
    vicon_tx_poses, _ = aggregate_tracker(vicon_tracked_uwb1_to_uwb1_tx, np.array(vicon_data["UWB1"]))

    vicon_uwbtx_json = [ {
        "t": float(body_pose[0]),
        "type": "vicon_tx_pose",
        "src": SOURCE,
        "T_body_world" : body_pose[1:].reshape((4,4)),
    } for body_pose in list(vicon_tx_poses)]

### Write Infra1 frames to output directory, and provide references in all_data
infra1_json = aggregate_infra1(topic_to_processing, out_infra1)

### Write Infra2 frames to output directory, and provide references in all_data
infra2_json = aggregate_infra2(topic_to_processing, out_infra2)

### Add synthetic ranges inter-anchor, since we weren't able to record the raw ranges

# ids = [1,3,4]
ids = [2,3,4] # For IRL3 trials, NUC2 was connected to anchor ID=1
f_uwb = 5 # 5hz
dt_uwb = 1/f_uwb

synth_inter_anchor_ranges = []
for src in ids:
    for dst in [a for a in ids if a != src]:

        timestamps = np.arange(START, END, dt_uwb)
        N_ranges = timestamps.shape[0]

        # Generate Gaussian-distributed ranges
        source = f"UWB{src}"
        dest = f"UWB{dst}"

        # Vicon_data is TUM formatted body_to_world
        # MAKE SURE THE INDEX YOU PICK IS RIGHT
        # If the 0th pose is bugged then all of the synthetic data will be wrong
        source_pose = slam_quat_to_HTM(vicon_data[source][0]) # UWB -> vicon world
        dest_pose = slam_quat_to_HTM(vicon_data[dest][0])

        source_position = T.T_vuwb_to_uwbtx @ np.linalg.inv(source_pose) # T_vuwb_to_tx X T_vworld_to_vuwb T_vworld_to_tx
        dest_position = T.T_vuwb_to_uwbtx @ np.linalg.inv(dest_pose)

        dist = np.linalg.norm(source_position - dest_position) # Compute distances between transmitters in the vicon world frame.
        stdev = 0.2
        ranges = np.random.normal(loc=dist, scale=stdev, size=N_ranges)

        ## NEEED to label src and dest ids now.
        synth_ranges = []
        for i in range(ranges.shape[0]):
            j = {
                "t":timestamps[i],
                "type": "uwb",
                "tag":"synth_for_anchors",
                "src": src,
                "id": dst,
                "range": ranges[i]
            }

            synth_ranges.append(j)
        synth_inter_anchor_ranges = synth_inter_anchor_ranges + synth_ranges



    # Compose the final factor graph dataset
all_data = uwb_json + imu_json + infra1_json + infra2_json + vicon_json + slam_json + assisted_uwb_json + vicon_uwbtx_json + synth_inter_anchor_ranges

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

### Copy all world information: transforms, anchors, apriltags, to output

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
print("Checking frequency of real data")
print(f" Measured UWB frequency {uwb_message_count / (END-START)}")
print(f" Measured vicon frequency {len(vicon_data['LeftRS']) / (END-START)}")

if args.slam_available:
    print(f" Measured SLAM frequency {len(slam_json) / (END-START)}")

# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])
json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

# All data synthetic is (all real data except slam) + (real SLAM (filtered) + synthetic UWB (created from interpolating on real SLAM))
all_data_synthetic = filtt( [a for a in all_data if not a["type"] == "slam_pose"] + all_data_synthetic) 
all_data_synthetic = sorted(all_data_synthetic, key=lambda x: x["t"])

json.dump(all_data_synthetic, open(outpath+"/synthetic"+f"/all_synthetic_{args.synth_slam_f}_{args.synth_uwb_f}.json", 'w'), cls=NumpyEncoder, indent=1)
# So all synthetic files will have a unique name


json.dump(args.__dict__, open(out_synthetic+f"/all_synthetic_{args.synth_slam_f}_{args.synth_uwb_f}_meta.json", 'w'), cls=NumpyEncoder, indent=1)
