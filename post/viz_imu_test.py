
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

import matplotlib.pyplot as plt


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


t, xs, ys, zs = ([], [], [] , [])
# I set it to unify accel and gyro, does unified accel and gyro go to the accel topic?
def proc_imu(msg):

    # I should be looking at a topic called 'imu' -> Interesting, I think I forgot to listen to this topic.
    # Despite unite_imu being set to 2, there is no 'imu' topic available in the ros2 topics list

    # print(msg)

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

    xs.append(msg.linear_acceleration.x)
    ys.append(msg.linear_acceleration.y)
    zs.append(msg.linear_acceleration.z)
    t.append(timestamp)

    return {"t":timestamp, "type":"imu", "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z, 
            "gx":msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz":msg.angular_velocity.z}




topic_to_processor_lambda = {
                  '/camera/camera/imu': proc_imu, 
}

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


    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 6), sharex=True)
    fig.suptitle("Axial acceleration in Realsense ROS coordinate frame")
    axes[0].plot(t, xs, color='r')
    axes[0].set_ylabel('Acc X (m/s²)')
    axes[0].set_title('Acceleration Over Time')

    axes[1].plot(t, ys, color='g')
    axes[1].set_ylabel('Acc Y (m/s²)')

    axes[2].plot(t, zs, color='b')
    axes[2].set_ylabel('Acc Z (m/s²)')
    axes[2].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()




    # json.dump(all_data, open(outpath+"/all.json", 'w'), indent=1)


