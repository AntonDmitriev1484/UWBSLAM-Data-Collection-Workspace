
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

from utils.load_rostypes import *
from utils.ros_msg_handlers import *


from pupil_apriltags import Detector



parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
args = parser.parse_args()


args.trial_name = "pilot3_slow_low"


outpath = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post'
out_rgb = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/rgb'
out_depth = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/depth'

out_gt = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_gt.json' # Standalone JSON file for GT
in_gt = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/thinkpad_ros_orbslam3_out/orb_slam3_ros_out/pilot3_slow_low_cam_traj.txt'

out_kf = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_kf.json' # Standalone JSON file for GT
in_kf = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/thinkpad_ros_orbslam3_out/orb_slam3_ros_out/pilot3_slow_low_kf_traj.txt'


os.makedirs(outpath, exist_ok=True)
os.makedirs(out_rgb, exist_ok=True)
os.makedirs(out_depth, exist_ok=True)

bagpath = Path(f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}')



def quat_to_HTM(nparr):
    translation = nparr[1:4]
    quat = nparr[4:8]

    print()
    print(nparr)
    print(translation)
    print(quat)

    r = R.from_quat(quat)
    rotation_matrix = r.as_matrix()

    # Assemble homogeneous transformation matrix (4x4)
    H = np.eye(4)
    H[:3, :3] = rotation_matrix
    H[:3, 3] = translation

    return H


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



TAG_POSE = True
KALIBR_CAMIMU_PATH = f"../kalibr/camimu_out/{trial_name}-camchain-imucam.yaml"
with open(KALIBR_CAMIMU_PATH, 'r') as fs: calibration = yaml.safe_load(fs)
# Remember CAM0 corresponds to infra1
CAM0_INTRINSICS = tuple(calibration['cam0']['intrinsics'])
TAG_SIZE = 0.100 #10cm tags

def extract_AprilTag_pose(cam0_images):
    # Maybe I've just set up my detector to look for the wrong tag type?
    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    # Find the frame at the first SLAM KF timestamp. That will let us process the AprilTag at SLAM frame origin.
    # Remember that you should pass an array of nparray style images from cam0 for 'cam0_images'

    for image in cam0_images:
        detections = at_detector.detect(image, TAG_POSE, CAM0_INTRINSICS, TAG_SIZE)



topic_to_processor_lambda = {
                '/uwb_ranges': proc_range,
                  '/camera/camera/imu': proc_imu, 
                  '/camera/camera/color/image_raw': proc_rgb_frame, 
                  '/camera/camera/depth/image_rect_raw': proc_depth_frame, 
}

all_data = []

dataset_topics = [ k for k,v in topic_to_processor_lambda.items()]

gt_standalone = []

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=typestore) as reader:


    connections = [x for x in reader.connections if x.topic in dataset_topics]
    for connection, timestamp, rawdata in reader.messages(connections=connections):

        msg = reader.deserialize(rawdata, connection.msgtype)

        processed_msg = topic_to_processor_lambda[connection.topic](msg)
        all_data.append(processed_msg)

    START = reader.start_time * 1e-9
    END = reader.end_time * 1e-9
    print(f"ROS duration {START} - {END}")

    # Now open up and read the GT file into all_data and gt_standalone

    gt_data = np.loadtxt(in_gt) # Assuming its formatted as t, x, y, z, quat
    print(f"ROS start: {START} ORBSLAM3 start: {gt_data[0,0]*1e-9}")
    print(f"ROS end: {END} . ORBSLAM3 end: {gt_data[-1, 0]*1e-9}") 

    for i in range(gt_data.shape[0]):
        gt_json = proc_gt(gt_data[i,:])
        all_data.append(gt_json) # Append GT data into the sensor stream to use as Pose3 corrections
        gt_standalone.append(gt_json) # Append GT into a standalone file if you want an independent fileread GT trajectory


    kf_data = np.loadtxt(in_kf)
    for i in range(kf_data.shape[0]):
        kf_json = proc_kf(kf_data[i,:])
        all_data.append(kf_json) # Append GT data into the sensor stream to use as Pose3 corrections
        kf_standalone.append(kf_json) # Append GT into a standalone file if you want an independent fileread GT trajectory


    # Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
    all_data = list(filter(lambda x: (START <= x["t"] <= END), all_data))
    all_data = sorted(all_data, key=lambda x: x["t"])

    json.dump(all_data, open(outpath+"/all.json", 'w'), indent=1)
    json.dump(gt_standalone, open(out_gt, 'w'), indent=1)
