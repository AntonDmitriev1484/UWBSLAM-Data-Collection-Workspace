
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

args = parser.parse_args()


outpath = f'./out/{args.trial_name}_post'
out_rgb = f'./out/{args.trial_name}_post/rgb'
out_depth = f'./out/{args.trial_name}_post/depth'
out_infra1 = f'./out/{args.trial_name}_post/infra1'
out_infra2 = f'./out/{args.trial_name}_post/infra2'
out_ml = f'./out/{args.trial_name}_post/ml'
os.makedirs(outpath, exist_ok=True)
os.makedirs(out_rgb, exist_ok=True)
os.makedirs(out_depth, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_ml, exist_ok=True)

in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_kalibr = f"../kalibr/camimu_out/{args.calibration_file}-camchain-imucam.yaml"
in_apriltags = f"../world/{args.apriltags_file}"
in_anchors = f"../world/{args.apriltags_file}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

slam_kf_data = np.loadtxt(in_slam_kf)
slam_kf_data[:,0] *= 1e-9
slam_data = np.loadtxt(in_slam)
slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'


def proc_gt(nparr):
    pose_slam_frame = quat_to_HTM(nparr)
    # TODO: SLAM_TO_WORLD_FRAME
    SLAM_TO_WORLD_FRAME = None
    pose_world_frame = SLAM_TO_WORLD_FRAME * pose_slam_frame

    timestamp = (nparr[0] * 1e-9)
    #return {"t":timestamp, "type":"gt_pose", "x": nparr[1], "y": nparr[2], "z":nparr[3], "qx": nparr[4], "qy":nparr[5], "qz":nparr[6], "qw":nparr[7]}
    return {"t":timestamp, "pose_slam": pose_slam_frame, "pose_world": pose_world_frame}

def proc_kf(nparr):
    # TODO: What frame should the IMU be in? Body or world?
    # Should I include the pose data estimated by my tracker?
    # I guess the more the better.
    # In that case, it might be better to do all of this in the output of my tracker graph.


    pose_slam_frame = quat_to_HTM(nparr)
    pose_world_frame = SLAM_TO_WORLD_FRAME * pose_slam_frame

    timestamp = (nparr[0] * 1e-9)
    # return {"t":timestamp, "type":"kf_pose", "x": nparr[1], "y": nparr[2], "z":nparr[3], "qx": nparr[4], "qy":nparr[5], "qz":nparr[6], "qw":nparr[7]}

# TODO: 
# Mark the timestamp of the first cam pose (0,0,0)
# Find the closest frame that gets the keyframe recognized to that timestamp
# Return the apriltag transform from that frame.




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

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=load_rostypes()) as reader:
    connections = [x for x in reader.connections if x.topic in dataset_topics]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        # if timestamp * 1e-9 >= ZERO_TIMESTAMP: # Cut raw data stream to start when ORBSLAM produces its first estimate
        msg = reader.deserialize(rawdata, connection.msgtype)
        proc, arr_ref = topic_to_processing[connection.topic]
        #all_data.append(proc(msg, arr_ref))
        proc(msg, arr_ref)


Transforms = SimpleNamespace()
infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]
Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)




START = reader.start_time * 1e-9
END = reader.end_time * 1e-9
print(f"ROS duration {START} - {END}")

# Now open up and read the GT file into all_data and gt_standalone

gt_data = np.loadtxt(in_slam) # Assuming its formatted as t, x, y, z, quat
print(f"ROS start: {START} ORBSLAM3 start: {gt_data[0,0]*1e-9}")
print(f"ROS end: {END} . ORBSLAM3 end: {gt_data[-1, 0]*1e-9}") 

for i in range(gt_data.shape[0]):
    gt_json = proc_gt(gt_data[i,:])
    all_data.append(gt_json) # Append GT data into the sensor stream to use as Pose3 corrections
    gt_standalone.append(gt_json) # Append GT into a standalone file if you want an independent fileread GT trajectory


for i in range(kf_data.shape[0]):
    kf_json = proc_kf(kf_data[i,:])
    all_data.append(kf_json) # Append GT data into the sensor stream to use as Pose3 corrections
    kf_standalone.append(kf_json) # Append GT into a standalone file if you want an independent fileread GT trajectory


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = list(filter(lambda x: (START <= x["t"] <= END), all_data))
all_data = sorted(all_data, key=lambda x: x["t"])

json.dump(all_data, open(outpath+"/all.json", 'w'), indent=1)
json.dump(gt_standalone, open(out_gt, 'w'), indent=1)
