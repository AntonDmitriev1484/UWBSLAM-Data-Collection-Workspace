
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
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R


from pupil_apriltags import Detector


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
args = parser.parse_args()


args.trial_name = "cam_target_stereo2"


# outpath = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post'
# out_rgb = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/rgb'
# out_depth = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_post/depth'

# out_gt = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_gt.json' # Standalone JSON file for GT
# in_gt = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/thinkpad_ros_orbslam3_out/orb_slam3_ros_out/pilot3_slow_low_cam_traj.txt'

# out_kf = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/{args.trial_name}_kf.json' # Standalone JSON file for GT
# in_kf = f'/home/admi3ev/ROS-Realsense-Decawave-Collector/thinkpad_ros_orbslam3_out/orb_slam3_ros_out/pilot3_slow_low_kf_traj.txt'


# os.makedirs(outpath, exist_ok=True)
# os.makedirs(out_rgb, exist_ok=True)
# os.makedirs(out_depth, exist_ok=True)

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


# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)


# Trying to see if we get detect apriltag pose from my stereo calibration bag
# cam_target_stereo2


def proc_rgb_frame(msg):
    #rgb8 encoding

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    # Make new file in out_rgb, labeled with timestamp.
    #img = arr.open('image_y8.png').convert('L')
    img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width))
    name = str(timestamp)+".png"
    return {"image": img_np}
    #cv2.imwrite(out_rgb+"/"+name, cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)) # Not exactly sure what cvtColor does...


topic_to_processor_lambda = {
                  '/camera/camera/infra1/image_rect_raw': proc_rgb_frame
}
all_data = []
dataset_topics = [ k for k,v in topic_to_processor_lambda.items()]

with AnyReader([bagpath], default_typestore=typestore) as reader:


    connections = [x for x in reader.connections if x.topic in dataset_topics]
    for connection, timestamp, rawdata in reader.messages(connections=connections):

        msg = reader.deserialize(rawdata, connection.msgtype)

        processed_msg = topic_to_processor_lambda[connection.topic](msg)
        all_data.append(processed_msg)


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

# estimate_tag_pose: bool = False, 
# camera_params: Optional[Tuple[float, float, float, float]] = None, 
# tag_size: Optional[float] = None

# These can all be read from the imucam.yaml file
estimate_tag_pose = True
camera_params = ( 382.3097047771611, 381.06337025907686, 320.267501168262, 241.70101347358377) # Manually copying in from calibration YAML
tag_size = 0.0205

for i, j in enumerate(all_data):

    image = j["image"]
    detections = at_detector.detect(image, estimate_tag_pose, camera_params, tag_size)

    cv2.imshow('Frame',image)
    cv2.waitKey(int((1/30) * 1000))

    if len(detections) > 0:
        print(detections)
        corners = detections[0].corners
        plt.scatter(corners[:,0], corners[:,1], c='red', s=3)
        plt.imshow(image)
        plt.title(f"Frame no: {i}")
        plt.show()