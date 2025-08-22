
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
from scipy.linalg import orthogonal_procrustes
from types import SimpleNamespace

import shutil
import math
import copy

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *

from scipy.optimize import minimize


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401



# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--calibration_file", "-c", type=str)
parser.add_argument("--crop_start", type=float) # Pass the ROS timestamp that you want to crop away all data before. Data will still be used to compute transforms.
parser.add_argument("--alias", type=str) # Alias for a dataset, ex. if you want to keep a cropped and uncropped version of a dataset
parser.add_argument("--anchors_file", "-a", type=str)
parser.add_argument("--apriltags_file", "-p", type=str)
parser.add_argument("--interpolate_slam", "-i", default=0, type=int) # -i controls how many interpolated poses you want between each pair of SLAM poses.
parser.add_argument("--synthetic_uwb_frequency", default=0, type=int) # interpolate GT to this frequency, so that gtsam_test can use synthetic ranges.
parser.add_argument("--synthetic_slam_frequency", default=0, type=int) #  filter GT to this frequency, must be < 20 should really be named 'lower_slam_frequency'
parser.add_argument("--real_uwb_orientation_support", default=True, type=bool)
parser.add_argument("--override_april_start", type=str )
# With real UWB ranges, but no compass, attach a pose to each uwb measurement using interpolation.
# Setting this flag interpolates N=100 poses between each SLAM pose, and maps them onto the temporally closest UWB range.


args = parser.parse_args()

outpath = f'./out/{args.trial_name}_post'
if args.alias is not None: outpath = f'./out/{args.alias}_post'

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



# Filter for messages within bag timestamp range.
START = reader.start_time * 1e-9
END = reader.end_time * 1e-9
print(f"ROS duration {START} - {END}")
print(f"Data start {START} cropped to {args.crop_start}")

def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


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

