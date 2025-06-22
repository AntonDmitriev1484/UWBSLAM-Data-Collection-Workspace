
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


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
args = parser.parse_args()


outpath = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post'
out_rgb = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/rgb'
out_depth = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/depth'

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_rgb, exist_ok=True)
os.makedirs(out_depth, exist_ok=True)

bagpath = Path(f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}')

add_types = {}

# Guide for handling types external to main ROS
# https://ternaris.gitlab.io/rosbags/topics/typesys.html

# Add Beluga custom message types
beluga_msg_dir = '/home/admi3ev/Beluga-2.0/ROS/src/beluga_messages/msg'
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

def proc_range(msg):
    msg = msg.ranges[0]
    timestamp = msg.timestamp.sec + (msg.timestamp.nanosec * 1e-9)
    return {"t":timestamp, "type":"uwb", "id":msg.id, "range":msg.range, "exchange": msg.exchange}

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


# I set it to unify accel and gyro, does unified accel and gyro go to the accel topic?
def proc_imu(msg):

    # I should be looking at a topic called 'imu' -> Interesting, I think I forgot to listen to this topic.
    # Despite unite_imu being set to 2, there is no 'imu' topic available in the ros2 topics list

    # print(msg)

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    return {"t":timestamp, "type":"imu", "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z, 
            "gx":msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz":msg.angular_velocity.z}

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

    # Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
    all_data = list(filter(lambda x: (START <= x["t"] <= END), all_data))
    all_data = sorted(all_data, key=lambda x: x["t"])

    json.dump(all_data, open(outpath+"/all.json", 'w'), indent=1)
