
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from rosbags.typesys import get_types_from_idl, get_types_from_msg

import pkgutil
import importlib
import inspect
import os
import json
import argparse

import cv2
import numpy as np

import csv

from scipy.spatial.transform import Rotation as R


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

add_types = {}

# Guide for handling types external to main ROS
# https://ternaris.gitlab.io/rosbags/topics/typesys.html

# Add Beluga custom message types
beluga_msg_dir = '/home/admi3ev/Beluga-Firmware-Mod/ROS/src/beluga_messages/msg'
for msg_name in os.listdir(beluga_msg_dir):
    filepath = beluga_msg_dir+f"/{msg_name}"

    msg_definition = Path(filepath).read_text()
    msg_name = f"beluga_messages/msg/{msg_name.removesuffix('.msg')}"
    add_types.update(get_types_from_msg(msg_definition, msg_name))

# Add Realsense custom message types
realsense_msg_dir = '/opt/ros/humble/share/realsense2_camera_msgs/msg'
for msg_name in os.listdir(realsense_msg_dir):
    if '.msg' in msg_name:
        filepath = realsense_msg_dir+f"/{msg_name}"
        msg_definition = Path(filepath).read_text()
        msg_name = f"realsense2_camera_msgs/msg/{msg_name.removesuffix('.msg')}"
        add_types.update(get_types_from_msg(msg_definition, msg_name))

print(add_types)

# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

def proc_range(msg): #TODO update this to new UWB topic
    msg = msg.ranges[0]
    timestamp = msg.timestamp.sec + (msg.timestamp.nanosec * 1e-9)

    j = {
        "t":timestamp,
        "type": "uwb",
        "id": msg.id,
        "range": msg.range,
        "exchange": msg.exchange,
        "maxnoise": msg.maxnoise,
        "firstpathamp1": msg.firstpathamp1,
        "firstpathamp2": msg.firstpathamp2,
        "firstpathamp3": msg.firstpathamp3,
        "stdnoise": msg.stdnoise,
        "maxgrowthcir": msg.maxgrowthcir,
        "rxpreamcount": msg.rxpreamcount,
        "firstpath": msg.firstpath
    }

    print(j)

    return j

def proc_rgb_frame(msg):
    #rgb8 encoding

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    # Make new file in out_rgb, labeled with timestamp.
    img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    name = str(timestamp)+".png"
    cv2.imwrite(out_rgb+"/"+name, cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)) # Not exactly sure what cvtColor does...

    return {"t":timestamp, "type":"rgb", "name":name}

def proc_depth_frame(msg):
    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    img_np = np.frombuffer(arr, dtype=np.uint16).reshape((msg.height, msg.width)) # Output says unit8 but encoding says 16UC1
    name = str(timestamp)+".png"
    cv2.imwrite(out_depth+"/"+name, img_np)

    return {"t":timestamp, "type":"depth", "name":name}


imu_count = 0
# I set it to unify accel and gyro, does unified accel and gyro go to the accel topic?
def proc_imu(msg):

    # I should be looking at a topic called 'imu' -> Interesting, I think I forgot to listen to this topic.
    # Despite unite_imu being set to 2, there is no 'imu' topic available in the ros2 topics list

    # print(msg)
    global imu_count 
    imu_count +=1

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    return {"t":timestamp, "type":"imu", "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z, 
            "gx":msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz":msg.angular_velocity.z}

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

SLAM_TO_WORLD_FRAME = 

def proc_gt(nparr):
    pose_slam_frame = quat_to_HTM(nparr)
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


topic_to_processor_lambda = {
                '/uwb_ranges': proc_range,
                  '/camera/camera/imu': proc_imu, 
                  '/camera/camera/color/image_raw': proc_rgb_frame, 
                  '/camera/camera/depth/image_rect_raw': proc_depth_frame, 
}
# TODO: We can add these later? Transforms between sensors on realsense I hope?
# /camera/camera/extrinsics/depth_to_accel \
# /camera/camera/extrinsics/depth_to_color \
# /camera/camera/extrinsics/depth_to_gyro \
# /camera/camera/color/metadata seems to contain very useful information, but I'm not sure how to use it at the moment lol



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
    # This verifies that the timestamps from the ORBSLAM3 output trajectory are in the same timeframe as the timestamps from the original data.
    print(f" ROS imu points {imu_count} ORBSLAM3 gt points: {gt_data.shape[0]}")
    # So it seems like GT was generated at 20Hz, where IMU runs at 200Hz
    # Rather than doing any kind of interpolation on GT, we're just going to add it into the datastream
    # and feed in a GT pose whenever it appears chronologically.

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
