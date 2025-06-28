
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
import yaml

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


from scipy.spatial.transform import Rotation as R

from utils.load_rostypes import *
from dt_apriltags import Detector

import pickle

from types import SimpleNamespace

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, '__dict__'):
            return vars(obj)
        return super().default(obj)
    

import os
import shutil

def clear_directory(dir_path):
    for filename in os.listdir(dir_path):
        file_path = os.path.join(dir_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)  # Remove file or symbolic link
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)  # Remove directory and its contents
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")


def draw_detection(frame, detection, out_path, CAM1_INTRINSICS):
        corners = detection.corners
        pose_R = detection.pose_R
        pose_t = detection.pose_t

        # Project axes into image
        axis_length = 0.3  # meters
        tip_length = 0.15

        # Define axes in 3D
        origin_3d = pose_t
        axes_3d = np.array([
            origin_3d,                             # origin
            origin_3d + pose_R @ np.array([[axis_length], [0], [0]]),  # X-axis
            origin_3d + pose_R @ np.array([[0], [axis_length], [0]]),  # Y-axis
            origin_3d + pose_R @ np.array([[0], [0], [axis_length]])   # Z-axis
        ])

        # Project to image plane
        camera_matrix = np.array([
            [CAM1_INTRINSICS[0], 0, CAM1_INTRINSICS[2]],
            [0, CAM1_INTRINSICS[1], CAM1_INTRINSICS[3]],
            [0, 0, 1]
        ])
        dist_coeffs = np.zeros(5)  # Assuming undistorted

        imgpts, _ = cv2.projectPoints(axes_3d, np.zeros((3,1)), np.zeros((3,1)), camera_matrix, dist_coeffs)
        imgpts = imgpts.astype(int).reshape(-1, 2)

        img = cv2.cvtColor(frame["raw"], cv2.COLOR_GRAY2BGR)

        origin = tuple(imgpts[0])
        cv2.arrowedLine(img, origin, tuple(imgpts[1]), (0, 0, 255), 2, tipLength=tip_length)  # X - red
        cv2.arrowedLine(img, origin, tuple(imgpts[2]), (0, 255, 0), 2, tipLength=tip_length)  # Y - green
        cv2.arrowedLine(img, origin, tuple(imgpts[3]), (255, 0, 0), 2, tipLength=tip_length)  # Z - blue

        # Save image
        out_path = out_path + f"/apriltag_pose_{frame['name']}.png"
        cv2.imwrite(out_path, img)
        print(f"Saved AprilTag pose image to {out_path}")

def extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags):
    ### The SLAM frame is defined at the starting pose of the IMU in the world frame.
    ### My body frame, is defined as a rotation out of the IMU frame.

    ZERO_TIMESTAMP = slam_data[0][0]

    TAG_POSE = True
    with open(in_kalibr, 'r') as fs: calibration = yaml.safe_load(fs)
    # Remember CAM0 corresponds to infra1
    CAM1_INTRINSICS = tuple(calibration['cam0']['intrinsics'])
    TAG_SIZE = 0.100 #10cm tags

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    closest_raw_frame = None
    detection = None

    # Ah I see, the first frame we recognized is actually earlier than when
    # SLAM is able to initialize tracking
    # SLAM takes ~3seconds before it outputs an estimate trajectory
    # Should I just crop the entire dataset to only begin once GT starts?

    # If we start the dataset when SLAM starts producing estimates, we risk completely missing the apriltag.
    # what the hell lol.
    # Why does SLAM wait 3 seconds to produce trajectory estimates?????
    # I guess I need to account for this in future dataset runs...

    # infra1_raw_frames_cropped = [ f for f in infra1_raw_frames if f["t"] > f[""]]

    # Find the "closest recognition before the first SLAM cam timestamp"

    detect_dbg_path = "/home/admi3ev/ws/post/debug/detection_frames/"
    clear_directory(detect_dbg_path)

    print(f"Read intrinsics {CAM1_INTRINSICS}")

    detections = []
    for frame in infra1_raw_frames:
        detections = at_detector.detect(frame["raw"], TAG_POSE, CAM1_INTRINSICS, TAG_SIZE)

        if len(detections) > 0:
            closest_raw_frame = frame
            draw_detection(frame, detection, detect_dbg_path, CAM1_INTRINSICS)
        
        if frame["t"] > ZERO_TIMESTAMP: break
    
    print(f"SLAM origin t={ZERO_TIMESTAMP}. Closest camera frame t={closest_raw_frame['t']}")


    with open(in_apriltags, 'r') as fs: apriltag_world_locations = json.load(fs)

    # Syntax T_a_b is "pose of a in frame b"

    rs_frame_dbg = SimpleNamespace()


    T_tag_cam1 = np.eye(4)
    print("Detection")
    print(detection)

    # So to average out the results of multiple detections,
    # You need to compute a T_slam_world per transform, and then average those.

    tag_cam1_candidates = []
    for d in detections:
        T_tag_cam1[:3, :3] = d.pose_R
        T_tag_cam1[:3, 3] = d.pose_t.flatten()
        Transforms.T_tag_cam1 = T_tag_cam1
    

    T_april_cam1 = np.array(apriltag_world_locations["T_april_cam1"])
    Transforms.T_april_cam1 = T_april_cam1

    # Throwing out the 'April' frame for now.
    # This detection is the location of the tag in apriltag coordinates
    # TODO find T_april_cam1, to transform from realsense coordinates to apriltag coordinates
    # TODO find T_april_world

    T_cam1_imu = np.array(calibration['cam0']['T_cam_imu'])
    Transforms.T_cam1_imu = T_cam1_imu
    # Transforms["T_apriltag_world"]

    DETECTED_ID = str(detection.tag_id)
    print(f" Detected tag_id {DETECTED_ID}")
    T_tag_world = np.array(apriltag_world_locations[DETECTED_ID]) # Get the world frame location of the center of the tag
    Transforms.T_apriltag_world = T_tag_world

    T_slam_world = T_tag_world @ np.linalg.inv( T_tag_cam1 @ T_cam1_imu) # Works?
    # T_slam_world = np.linalg.inv( T_tag_cam1 @ T_cam1_imu) @ T_tag_world # What I think is mathematically correct

    # T_slam_world =  T_tag_cam1 @ T_cam1_imu @ T_tag_world # Works?
    # T_slam_world = np.linalg.inv(T_cam1_imu) @ np.linalg.inv(T_tag_cam1) @ T_tag_world
    Transforms.T_slam_world = T_slam_world

    origin = np.eye(4)

    rs_frame_dbg.T_tag_cam1 = T_tag_cam1
    rs_frame_dbg.T_tag_imu = T_tag_cam1 @ T_cam1_imu
    rs_frame_dbg.T_imu_tag = np.linalg.inv(rs_frame_dbg.T_tag_imu)
    rs_frame_dbg.T_cam1_imu = T_cam1_imu

    world_frame_dbg = SimpleNamespace()
    world_frame_dbg.T_tag_world = T_tag_world
    world_frame_dbg.origin = origin
    world_frame_dbg.T_slam_world = T_slam_world

    print(Transforms)

    with open(f'/home/admi3ev/ws/post/debug/rs_frame_dbg.json', 'w') as fs: json.dump(vars(rs_frame_dbg),fs, cls=NumpyEncoder, indent=1)
    with open(f'/home/admi3ev/ws/post/debug/world_frame_dbg.json', 'w') as fs: json.dump(vars(world_frame_dbg),fs, cls=NumpyEncoder, indent=1)
    # debug_transforms(Transforms)


    return Transforms #TODO



if __name__ == "__main__":


    parser = argparse.ArgumentParser(description="Stream collector")
    parser.add_argument("--trial_name" , "-t", type=str)
    args = parser.parse_args()


    bagpath = Path(f'../collect/ros2/{args.trial_name}')
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

    with AnyReader([bagpath], default_typestore=load_rostypes()) as reader:


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