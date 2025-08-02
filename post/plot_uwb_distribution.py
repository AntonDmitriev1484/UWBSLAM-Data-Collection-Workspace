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
matplotlib.use('Agg')

import traceback
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)

args = parser.parse_args()
outpath = f'./out/{args.trial_name}_post'
os.makedirs(outpath, exist_ok=True)
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

        except Exception as e:
            print(f"\n[ERROR] Failed to process message on topic '{connection.topic}'")
            print(f"Message type: {connection.msgtype}")
            print(f"Exception: {e}")
            traceback.print_exc()

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
# Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

# Processors functions have now buffered their individual topics into arr_ref
# This is useful for writing the same datastream to multiple files.
# Then, lastly, we can create all.json using the buffered measurements.


### Write UWB data to its own csv file, and to all_data
uwb_csv = []
ranges_log = {}
for j in topic_to_processing['/uwb_ranges'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v) # This should iterate in the order of how keys are originally defined in the json
    uwb_csv.append(csv_row)
    all_data.append(j)
    if j['id'] not in ranges_log: ranges_log[j['id']] = []
    else: ranges_log[j['id']].append(j['range'])

from scipy.stats import norm


for anchor, uwb_range_distribution in ranges_log.items():
    # Fit a Gaussian to UWB range distribution
    range_array = np.array(uwb_range_distribution)
    mu, std = norm.fit(range_array)

    # Plot histogram
    plt.figure(figsize=(8, 5))
    n, bins, patches = plt.hist(range_array, bins=50, density=True, alpha=0.6, color='skyblue', edgecolor='black', label="UWB Ranges")

    # Plot the fitted Gaussian
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 500)
    p = norm.pdf(x, mu, std)
    plt.plot(x, p, 'r--', linewidth=2, label=f"Fitted Gaussian\n$\sigma$ = {std:.3f}\n$\mu$ = {mu:.3f}")

    # Labels and saving
    plt.title(f"UWB Range Distribution for Anchor {anchor}")
    plt.xlabel("Range (m)")
    plt.ylabel("Density")
    plt.legend()
    plt.grid(True)

    hist_outpath = f"{outpath}/uwb_range_histogram_anchor{anchor}.png"
    plt.savefig(hist_outpath)
    print(f"Saved histogram to {hist_outpath}")
    plt.close()
