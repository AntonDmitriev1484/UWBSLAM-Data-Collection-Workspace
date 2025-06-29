
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
from types import SimpleNamespace

import shutil

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *

# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--calibration_file", "-c", type=str)
parser.add_argument("--anchors_file", "-a", type=str)
parser.add_argument("--apriltags_file", "-p", type=str)
parser.add_argument("--interpolate_slam_hz", "-i", type=int)

args = parser.parse_args()


outpath = f'./out/{args.trial_name}_post'
out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_ml = f'{outpath}/ml'
os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_ml, exist_ok=True)

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
        # if timestamp * 1e-9 >= ZERO_TIMESTAMP: # Cut raw data stream to start when ORBSLAM produces its first estimate

        # if connection.msgtype != "beluga_messages/msg/BelugaRanges":

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
for j in topic_to_processing['/uwb_ranges'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v) # This should iterate in the order of how keys are originally defined in the json
    uwb_csv.append(csv_row)
    all_data.append(j)

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
for i in range(slam_data.shape[0]):

    T_body_slam = slam_quat_to_HTM(slam_data[i,:])
    slam_data_slam_frame.append( [slam_data[i,0]] + list(T_body_slam.flatten()) )

    # T_body_world = Transforms.T_slam_world @ T_body_slam
    T_body_world =  np.linalg.inv(Transforms.T_slam_world @ T_body_slam)
    slam_data_world_frame.append( [slam_data[i,0]] + list(T_body_world.flatten()) )


    if args.interpolate_slam_hz is not None:
        print(f"Interpolating SLAM trajectory to {args.interpolate_slam_hz}")

    j = {
        "t": slam_data[i,0],
        "type": "slam_pose",
        "T_body_slam" : T_body_slam,
        "T_body_world" : T_body_world
    }
    all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections

with open(f'{out_ml}/slam_data_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_data_world_frame))
with open(f'{out_ml}/slam_data_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_data_slam_frame))

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



### Write all world information: transforms, anchors, apriltags, to output
shutil.copy(in_anchors, f'{outpath}/anchors.json')
shutil.copy(in_apriltags, f'{outpath}/apriltags.json')
with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])

json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)
