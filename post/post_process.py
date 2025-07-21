
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

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--calibration_file", "-c", type=str)
parser.add_argument("--anchors_file", "-a", type=str)
parser.add_argument("--apriltags_file", "-p", type=str)
parser.add_argument("--interpolate_slam", "-i", default=0, type=int) # -i controls how many interpolated poses you want between each pair of SLAM poses.
parser.add_argument("--synthetic_uwb_frequency", default=0, type=int) # interpolate GT to this frequency, so that gtsam_test can use synthetic ranges.
parser.add_argument("--synthetic_slam_frequency", default=0, type=int) #  filter GT to this frequency, must be < 20

args = parser.parse_args()


outpath = f'./out/{args.trial_name}_post'
out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_ml = f'{outpath}/ml'
out_synthetic = outpath+"/synthetic"

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_ml, exist_ok=True)
os.makedirs(out_synthetic, exist_ok=True)

in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_kalibr = f"../kalibr/camimu_out/{args.calibration_file}-camchain-imucam.yaml"
in_apriltags = f"../world/{args.apriltags_file}"
in_anchors = f"../world/{args.anchors_file}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

slam_kf_data = np.loadtxt(in_slam_kf)
slam_kf_data[:,0] *= 1e-9
slam_data = np.loadtxt(in_slam)
slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'

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


ZERO_TIMESTAMP = slam_data[0][0]

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

print(f" Processed {processed_uwb_message} / {uwb_message_count} total messages")

# Filter for messages within bag timestamp range.
START = reader.start_time * 1e-9
END = reader.end_time * 1e-9
print(f"ROS duration {START} - {END}")
def filtt(arr): return list(filter(lambda x: (START <= x["t"] <= END), arr))
def filtt2(arr): return list(filter(lambda x: (START <= x[0] <= END), arr))


Transforms = SimpleNamespace()
infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]
Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

# Processors functions have now buffered their individual topics into arr_ref
# This is useful for writing the same datastream to multiple files.
# Then, lastly, we can create all.json using the buffered measurements.


### Write UWB data to its own csv file, and to all_data
uwb_csv = []
uwb_range_distribution = []
for j in topic_to_processing['/uwb_ranges'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v) # This should iterate in the order of how keys are originally defined in the json
    uwb_csv.append(csv_row)
    all_data.append(j)
    uwb_range_distribution.append(j['range'])

with open(f'{out_ml}/uwb_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(uwb_csv))

### Write IMU data to its own csv file, and to all_data
imu_csv = []
for j in topic_to_processing['/camera/camera/imu'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v)
    imu_csv.append(csv_row)
    all_data.append(j)
with open(f'{out_ml}/imu_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(imu_csv))

### Write SLAM camera trajectory
slam_data_world_frame = []
slam_data_slam_frame = []
slam_pose_counter = 0

all_data_synthetic = [] # Keep interpolated points in a separate file from all.json


dataset_slam_pose_frequency = 20 # Need something evenly divisible
if (args.synthetic_slam_frequency > dataset_slam_pose_frequency):
    print("Error: can't be doin that buddy")
    exit()

n_slam_skip = 0
if args.synthetic_slam_frequency > 0: n_slam_skip = int(dataset_slam_pose_frequency/args.synthetic_slam_frequency)

if args.synthetic_uwb_frequency > dataset_slam_pose_frequency:
    n_points = int(args.synthetic_uwb_frequency / dataset_slam_pose_frequency) # How much we need to interpolate between existing orbslam points to get this frequency of UWB
    n_skip = 1
else:
    n_points = 1
    n_skip = 0
    if args.synthetic_uwb_frequency > 0:
        n_skip = int(dataset_slam_pose_frequency / args.synthetic_uwb_frequency)
    # ex. with 20 hz GT, and we want to simulate 5Hz UWB, we only interpolate synthetic UWB between every 4th pose pair.

print(f"{n_points=} {n_skip=} {n_slam_skip=}")

if args.interpolate_slam > 0: print(f"Interpolating SLAM trajectory to {args.interpolate_slam=} .")

for i in range(slam_data.shape[0]-1):

    T_body_slam = slam_quat_to_HTM(slam_data[i,:])
    slam_data_slam_frame.append( [slam_data[i,0]] + list(T_body_slam.flatten()) )

    T_body_world = Transforms.T_slam_world @ T_body_slam
    slam_data_world_frame.append( [slam_data[i,0]] + list(T_body_world.flatten()) )

    j = {
        "t": slam_data[i,0],
        "type": "slam_pose",
        "T_body_slam" : T_body_slam,
        "T_body_world" : T_body_world
    }
    all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections
    slam_pose_counter += 1

    if n_slam_skip > 0 and (slam_pose_counter % n_slam_skip == 0): all_data_synthetic.append(j)

    if n_skip > 0 and (slam_pose_counter % n_skip == 0) and n_points > 0:
        # All in the slam frame first
        current_timestamp = slam_data[i, 0]
        current_pose = T_body_slam
        next_pose = slam_quat_to_HTM(slam_data[i+1,:])

        dTranslation = (next_pose[:3,3] - current_pose[:3,3]) / (n_points+1)
        dt = (slam_data[i+1,0] - slam_data[i, 0]) / (n_points+1)

        Rotato = next_pose[:3, :3]

        print(f" Between t: {slam_data[i+1, 0]} and {slam_data[i, 0]}")

        # Use Slerp to interpolate on SE(3) rotations
        interp_interval = [slam_data[i,0], slam_data[i+1, 0]]
        interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
        slurpy = Slerp(interp_interval, interp_rots)
        interp_timestamps = np.linspace(slam_data[i,0], slam_data[i+1, 0], n_points)
        interpolated_rotations = slurpy(interp_timestamps)

        # Use kinematics to interpolate on R3 positions
        for p in range(1, n_points+1):

            interp_slam_pose = np.eye(4)
            interp_slam_pose[:3, 3] = current_pose[:3, 3] + (dTranslation * p)
            # interp_slam_pose[:3, :3] = Rotation
            interp_slam_pose[:3,:3] = interpolated_rotations[p-1].as_matrix()

            interp_world_pose = Transforms.T_slam_world @ interp_slam_pose  

            interp_timestamp = current_timestamp + (p * dt)
            print(f" Interpolated timestamp is {interp_timestamp}")

            j = { # Note: Only going to interpolate into all.json because I just need this in the tracker.
                "t": interp_timestamp,
                "type": "synthetic_uwb",
                "T_body_slam" : interp_slam_pose,
                "T_body_world" : interp_world_pose
            }
            all_data_synthetic.append(j)


# Compute velocities in the world frame
# This way you can set a velocity prior at any time

np_slam_data_world_frame = np.array(slam_data_world_frame)
dt = np.diff(np_slam_data_world_frame[:,0])
dx = np.diff(np_slam_data_world_frame[:,4]) / dt
dy = np.diff(np_slam_data_world_frame[:,7]) / dt
dz = np.diff(np_slam_data_world_frame[:,10]) / dt
slam_data_velocity_world_frame = np.vstack((np_slam_data_world_frame[:np_slam_data_world_frame.shape[0]-1,0], dx, dy, dz)).T
# By default. I map the velocity between t and t+1 to timestamp t. This should be good enough for prioring.

# TODO: Verify this is computing the right thing and in the right frame.
# TODO: Decide on crop or not.
# print(slam_data_velocity_world_frame[:10])

print(f" SLAM world 0: {slam_data_world_frame[0][0]} SLAM velocity world 0: {slam_data_velocity_world_frame[0][0]}")

# For all slam poses
slam_idx = 0
for i, mes in enumerate(all_data):
    if mes["type"] == "slam_pose":
        mes_ = mes
        if slam_idx < slam_data_velocity_world_frame.shape[0]:
            mes_["v_world"] = {
                "vx": slam_data_velocity_world_frame[ slam_idx, 1],
                "vy": slam_data_velocity_world_frame[ slam_idx, 2],
                "vz": slam_data_velocity_world_frame[ slam_idx, 3]
            }
        all_data[i] = mes_ # Extend each pose to also include its computed velocity
        slam_idx +=1






with open(f'{out_ml}/slam_data_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_data_world_frame))
with open(f'{out_ml}/slam_data_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_data_slam_frame))

slam_data_world_frame_tum = []
# timestamps = []
# for row in slam_data_world_frame:
#     slam_data_world_frame_tum.append(slam_HTM_to_TUM(row))
#     timestamps.append(row[0])
# with open(f'{outpath}/timestamps.txt', 'w') as fs: csv.writer(fs, delimiter=' ').writerows(filtt2(slam_data_world_frame_tum))
with open(f'{outpath}/slam_data_world_frame_tum.txt', 'w') as fs: csv.writer(fs, delimiter=' ').writerows(filtt2(slam_data_world_frame_tum))

### Write SLAM KF trajectory

slam_kf_data_world_frame = []
slam_kf_data_slam_frame = []
for i in range(slam_kf_data.shape[0]):

    T_body_slam = slam_quat_to_HTM(slam_kf_data[i,:])
    slam_kf_data_slam_frame.append( [slam_kf_data[i,0]] + list(T_body_slam.flatten()) )

    T_body_world = Transforms.T_slam_world @ T_body_slam

    slam_kf_data_world_frame.append( [slam_kf_data[i,0]] + list(T_body_slam.flatten()))

    j = {
        "t": slam_kf_data[i,0],
        "type": "slam_kf_pose",
        "T_body_slam" : T_body_slam,
        "T_body_world" : T_body_world
    }
    all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections


with open(f'{out_ml}/slam_kf_data_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_kf_data_world_frame))
with open(f'{out_ml}/slam_kf_data_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_kf_data_slam_frame))


### Write Infra1 frames to output directory, and provide references in all_data
for j in topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]:
    cv2.imwrite(out_infra1+"/"+j["name"], j["raw"])
    j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
    all_data.append(j_no_image)

### Write Infra2 frames to output directory, and provide references in all_data
for j in topic_to_processing['/camera/camera/infra2/image_rect_raw'][1]:
    cv2.imwrite(out_infra2+"/"+j["name"], j["raw"])
    j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
    all_data.append(j_no_image)



class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, '__dict__'):
            return vars(obj)
        return super().default(obj)



### Copy all world information: transforms, anchors, apriltags, to output
shutil.copy(in_anchors, f'{outpath}/anchors.json')
shutil.copy(in_apriltags, f'{outpath}/apriltags.json')
with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])
json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

# All data syntehtic is real IMU + (real SLAM (filtered) + synthetic UWB (created from interpolating on real SLAM))
all_data_synthetic = filtt( [a for a in all_data if a["type"] == "imu"] + all_data_synthetic) 
all_data_synthetic = sorted(all_data_synthetic, key=lambda x: x["t"])

json.dump(all_data_synthetic, open(outpath+"/synthetic"+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}.json", 'w'), cls=NumpyEncoder, indent=1)
# So all synthetic files will have a unique name

print("Checking frequency of synthetic data")

nuwb, ngt = (0,0)
for mes in all_data_synthetic:
    if mes["type"] == "synthetic_uwb": nuwb+=1
    if mes["type"] == "slam_pose": ngt+=1

generated_fuwb = nuwb / (END-START)
generated_fgt = ngt / (END-START)

print(f" UWB requested f={args.synthetic_uwb_frequency} , generated f={generated_fuwb}")
print(f" GT requested f={args.synthetic_slam_frequency} , generated f={generated_fgt}")

to_dump = {
    "meta": {
        "slam_freq": args.synthetic_slam_frequency,
        "generated_slam_freq": generated_fgt,
        "uwb_freq": args.synthetic_uwb_frequency,
        "generated_uwb_freq": generated_fuwb
    }
}
json.dump(to_dump, open(out_synthetic+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}_meta.json", 'w'), cls=NumpyEncoder, indent=1)