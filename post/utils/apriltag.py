
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
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import



from scipy.spatial.transform import Rotation as R

from utils.load_rostypes import *
from dt_apriltags import Detector
from utils.math_utils import * 

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
    corners = detection.corners  # shape: (4, 2)
    pose_R = detection.pose_R    # shape: (3, 3)
    pose_t = detection.pose_t    # shape: (3, 1)

    axis_length = 0.3  # meters
    tip_length = 0.15

    # Define 3D axes relative to pose
    axes_3d = np.array([
        [0, 0, 0],
        [axis_length, 0, 0],
        [0, axis_length, 0],
        [0, 0, axis_length]
    ]).reshape(-1, 3)

    # Camera intrinsics
    camera_matrix = np.array([
        [CAM1_INTRINSICS[0], 0, CAM1_INTRINSICS[2]],
        [0, CAM1_INTRINSICS[1], CAM1_INTRINSICS[3]],
        [0, 0, 1]
    ])
    dist_coeffs = np.zeros(5)  # Assuming undistorted image

    # Project axes
    imgpts, _ = cv2.projectPoints(axes_3d, cv2.Rodrigues(pose_R)[0], pose_t, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int).reshape(-1, 2)

    img = cv2.cvtColor(frame["raw"], cv2.COLOR_GRAY2BGR)

    # Draw AprilTag corner points
    for pt in corners:
        pt = tuple(map(int, pt))
        cv2.circle(img, pt, 4, (0, 255, 255), -1)  # yellow

    # Draw pose axes
    origin = tuple(imgpts[0])
    cv2.arrowedLine(img, origin, tuple(imgpts[1]), (0, 0, 255), 2, tipLength=tip_length)   # X - red
    cv2.arrowedLine(img, origin, tuple(imgpts[2]), (0, 255, 0), 2, tipLength=tip_length)   # Y - green
    cv2.arrowedLine(img, origin, tuple(imgpts[3]), (255, 0, 0), 2, tipLength=tip_length)   # Z - blue

    # Show image in a popup
    # window_name = f"AprilTag Pose - {frame['name']}"
    # cv2.imshow(window_name, img)
    # cv2.waitKey(0)
    # cv2.destroyWindow(window_name)

    # Save image
    full_path = f"{out_path}/apriltag_pose_{detection.tag_id}.png"
    cv2.imwrite(full_path, img)
    print(f"Saved AprilTag pose image to {full_path}")

def extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags):

    ZERO_TIMESTAMP = slam_data[0][0]

    TAG_POSE = True
    with open(in_kalibr, 'r') as fs: calibration = yaml.safe_load(fs)
    # Remember CAM0 corresponds to infra1
    CAM1_INTRINSICS = tuple(calibration['cam0']['intrinsics'])
    TAG_SIZE = 0.200 #20cm tags

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    #

    closest_raw_frame = None
    detection = None

    detect_dbg_path = "/home/admi3ev/ws/post/debug/detection_frames/"
    clear_directory(detect_dbg_path)

    print(f"Read intrinsics {CAM1_INTRINSICS}")

    # Make a list of all detected apriltags in the motion.
    all_detections = [] # (detections obj, associated_frame)

    for frame in infra1_raw_frames:
        detections_ = at_detector.detect(frame["raw"], TAG_POSE, CAM1_INTRINSICS, TAG_SIZE)
        if len(detections_) > 0:
            all_detections.append((detections_, frame))

    # First pick 20 candidates with the highest decision margin (higher is better)
    # x[0][0] because x[0] is an array of multiple detections
    best_detections =  (list(sorted(all_detections, key=lambda x: x[0][0].decision_margin, reverse=True)))[:20]

    # Then of those, pick the detections with the lowest delay
    lowest_match_delay = 10000
    best_match = None # (detections obj, associated frame, associated pose)
    for (detections, frame) in best_detections:

        frame_t = frame["t"]
        closest_slam_pose_index = np.argmin(np.abs(slam_data[:,0] - frame_t))
        slam_pose = slam_data[closest_slam_pose_index, :]
        slam_t = slam_pose[0]

        match_delay = abs(slam_t - frame_t)
        if (match_delay < lowest_match_delay):
            lowest_match_delay = match_delay
            best_match = (detections, frame, slam_pose)


    
    print(f"Best match \n {best_match=}")
    print(f"Time delay of {lowest_match_delay}s")

    for match in best_match[0]:
        print(match)
        draw_detection(best_match[1], match, detect_dbg_path, CAM1_INTRINSICS)

    with open(in_apriltags, 'r') as fs: apriltag_world_locations = json.load(fs)

    ### ORBSLAM3 outputs the pose of the left camera over time.
    ### Therefore frames 'cam1' and 'sbody' are analogous
    ### My body frame is defined as a rotation out of the IMU frame.

    # Note: Python '@' operator associates a chain in reverse of how you would write it out in right multiplication
    # T_a_to_b is the HTM that transforms a vector from frame A to frame B.

    T_cam1_to_tag = np.eye(4)
    # The detection we use should always be the one with the highest decision margin
    # detection = sorted(best_match[0], key=lambda x: x.decision_margin, reverse= True)[0]

    detection = None # Find best tag detection within our selected detection
    best_margin = 0
    for d in best_match[0]:
        if d.decision_margin > best_margin:
            best_margin = d.decision_margin
            detection = d
    print(f"Using detection {detection=}")

    T_cam1_to_tag[:3, :3] = detection.pose_R # Pose of tag in camera frame
    T_cam1_to_tag[:3, 3] = detection.pose_t.flatten()
    # T_cam1_to_tag[:3, 3] = -np.matmul(T_cam1_to_tag[:3, 3], -1 * detection.pose_t.flatten())
    pose_slam = slam_quat_to_HTM(best_match[2])
    T_sorigin_to_sbody = pose_slam # SLAM pose is the transform from the slam origin to slam body
    # The starting point of cam1 in space is the slam origin.


    DETECTED_ID = str(detection.tag_id)
    # T_world_to_tag = np.array(apriltag_world_locations[DETECTED_ID]) # Get the world frame location of the center of the tag
    # T_world_to_tag = np.linalg.inv(np.array(apriltag_world_locations[DETECTED_ID]))
    T_world_to_tag = np.array(apriltag_world_locations[DETECTED_ID])
    T_world_to_tag[:3,:3] = np.linalg.inv(T_world_to_tag[:3,:3]) # Ok yeah Apparently I just compute the rotation backwards always so inverting it is a must.
    # T_world_to_tag[:3,3] = np.matmul(T_world_to_tag[:3,:3] , -1 * T_world_to_tag[:3,3])

    # # How you would write it by hand (doesn't work)
    # # H_world_to_sorigin = np.linalg.inv(H_sorigin_to_sbody) @ np.linalg.inv(H_sbody_to_cam1) @ np.linalg.inv(H_cam1_to_tag) @ H_world_to_tag
    # H_world_to_sorigin = (
    #     H_world_to_tag
    #     @ np.linalg.inv(H_cam1_to_tag)
    #     @ np.linalg.inv(H_sbody_to_cam1)
    #     @ np.linalg.inv(H_sorigin_to_sbody)
    # )

    # How you would write it by hand (doesn't work)
    # T_world_to_sorigin = np.linalg.inv(T_sorigin_to_sbody) @ np.linalg.inv(T_cam1_to_tag) @ T_world_to_tag
    T_world_to_sorigin = (
        T_world_to_tag
        @ np.linalg.inv(T_cam1_to_tag)
        @ np.linalg.inv(T_sorigin_to_sbody)
    )

    Transforms.T_imu_to_cam1 = np.array(calibration['cam0']['T_cam_imu'])
    Transforms.T_imu_to_sbody = Transforms.T_imu_to_cam1
    Transforms.T_world_to_sorigin = T_world_to_sorigin

    Transforms.T_slam_world = T_world_to_sorigin 
    # Transforms uses older notation T_slam_world, transform from world to slam origin, i.e. poes of slam origin in world frame

    world_frame_dbg = SimpleNamespace()
    world_frame_dbg.H_world_to_sorigin = T_world_to_sorigin
    world_frame_dbg.H_world_to_tag = T_world_to_tag
    world_frame_dbg.origin = np.eye(4)
    world_frame_dbg.origin_to_aprilframe = T_cam1_to_tag @ np.eye(4) # Suppose the camera frame is at the origin, what is the reported aprilframe?

    print(Transforms)

    with open(f'/home/admi3ev/ws/post/debug/world_frame_dbg.json', 'w') as fs: json.dump(vars(world_frame_dbg),fs, cls=NumpyEncoder, indent=1)

    return Transforms


def extract_apriltag_pose_PnP(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags):

    TAG_POSE = True
    with open(in_kalibr, 'r') as fs: calibration = yaml.safe_load(fs)
    # Remember CAM0 corresponds to infra1
    CAM1_INTRINSICS = tuple(calibration['cam0']['intrinsics'])
    CAM1_INTRINSICS_MAT = np.array([
                                    [CAM1_INTRINSICS[0], 0, CAM1_INTRINSICS[2]],
                                    [0, CAM1_INTRINSICS[1], CAM1_INTRINSICS[3]],
                                    [0, 0, 1]
                                    ], dtype=np.float64)
    CAM1_DISTORTION = tuple(calibration['cam0']['distortion_coeffs'])
    CAM1_DISTORTION_VEC = np.array(CAM1_DISTORTION, dtype=np.float64)


    TAG_SIZE = 0.200 #20cm tags

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    
    detection = None

    detect_dbg_path = "/home/admi3ev/ws/post/debug/detection_frames/"
    clear_directory(detect_dbg_path)

    print(f"Read intrinsics {CAM1_INTRINSICS}")

    # Make a list of all detected apriltags in the motion.
    all_detections = [] # (detections obj, associated_frame)

    # infra1_raw_frames  = [f for f in infra1_raw_frames if f["t"] > 1750711255.2999153]

    for frame in infra1_raw_frames:
        detections_ = at_detector.detect(frame["raw"], TAG_POSE, CAM1_INTRINSICS, TAG_SIZE)
        if len(detections_) > 0:
            all_detections.append((detections_, frame))

    # Let the best match be defined as the one with the lowest time delay
    # Really, I think this should first be based on the one with highest detection certainty?

    # First pick 20 candidates with the highest total decision margin (higer is better)
    # Consider decision margin for both detections
    best_detections =  (list(sorted(all_detections, key=lambda detections_: sum([d.decision_margin for d in detections_[0]]), reverse=True)))[:20]

    # Test that this works by only making the selection be out of frames after timestamp 40?

    # Then of those, pick the ones with the best delay

    lowest_match_delay = 10000
    best_match = None # (detections obj, associated frame, associated pose)
    for (detections, frame) in best_detections:

        frame_t = frame["t"]
        # print(slam_data[:,0] - frame_t)
        closest_slam_pose_index = np.argmin(np.abs(slam_data[:,0] - frame_t))
        slam_pose = slam_data[closest_slam_pose_index, :]
        # slam_t = slam_data[np.argmin(slam_data[:,0] - frame_t), 0]
        slam_t = slam_pose[0]

        match_delay = abs(slam_t - frame_t)
        if (match_delay < lowest_match_delay):
            lowest_match_delay = match_delay
            best_match = (detections, frame, slam_pose)


    
    print(f"Best match \n {best_match=}")
    print(f"Time delay of {lowest_match_delay}s")

    for match in best_match[0]:
        draw_detection(best_match[1], match, detect_dbg_path, CAM1_INTRINSICS)

    # Now use PnP solver to compute the transform 'T_slam_world?' using multiple tags.
    # How to convert cam1 intrinsics vector to matrix?

    with open(in_apriltags, 'r') as fs: apriltag_world_locations = json.load(fs)

    half = TAG_SIZE / 2
        # Counterclockwise from bottom left point in world frame
    corners_in_tag_frame = np.array([
                                [-half,  half, 0],
                                [ half,  half, 0],
                                [ half, -half, 0],
                                [-half, -half, 0]
                            ], dtype=np.float64)

    # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d

    worldPoints = [] # 3d coordinates of each point in the world frame.
    imagePoints = [] # 2d coordinates of each point in the image frame

    # Ordering of corners in 'corners_in_tag_frame' needs to match up with ordering in 'detection.corners'
    # Also: It still seems like the transform of each tag frame point into the world frame is incorrect.

    for detection in best_match[0]:

        # This should be tag -> world frame, but we read world to tag
        H_world_to_tag = np.array(apriltag_world_locations[str(detection.tag_id)])
        # H_tag_to_world = np.linalg.inv(H_world_to_tag) 
        # # You'd think you'd have to invert it? But I guess that's not the case...
        # This gives the right matrix multiplication results in REPL?
        # maybe just ask Jose

        print(str(detection.tag_id))
        print(H_world_to_tag)

        for corner_tagframe, corner_imageframe in zip(corners_in_tag_frame, detection.corners):
            
            worldPoints.append(  (H_world_to_tag @ np.hstack([corner_tagframe, 1]))[:3]  ) # Append and then truncate a 1 from the vector
            imagePoints.append(corner_imageframe)

    
    worldPoints = np.ascontiguousarray(np.array(worldPoints, dtype=np.float64))
    imagePoints = np.ascontiguousarray(imagePoints, dtype=np.float64).reshape(len(imagePoints), 1, 2)

    print(worldPoints)
    print(imagePoints)

    #     # --- Plotting
    # fig, ax = plt.subplots(figsize=(8, 6))
    # ax.set_title("2D Image Points from AprilTag Detection")
    # ax.set_xlabel("Pixel X")
    # ax.set_ylabel("Pixel Y")
    # ax.invert_yaxis()  # Origin is usually top-left in images

    # for i, (img_pt, world_pt) in enumerate(zip(imagePoints, worldPoints)):
    #     x, y = img_pt[0]
    #     ax.plot(x, y, 'ro')  # Red dot
        
    #     ax.text(x + 3, y + 3, f'image ({img_pt[0][0]:.2f}, {img_pt[0][1]:.2f})\n world ({world_pt[0]:.2f}, {world_pt[1]:.2f}, {world_pt[2]:.2f})', fontsize=8)

    #     # Optional: draw bounding box edges (assumes 4 or 8 points in order)
    #     if len(imagePoints) == 4 or len(imagePoints) == 8:
    #         pts2d = np.squeeze(imagePoints, axis=1)
    #         for i in range(4):  # one tag
    #             ax.plot(
    #                 [pts2d[i][0], pts2d[(i+1)%4][0]],
    #                 [pts2d[i][1], pts2d[(i+1)%4][1]],
    #                 'b-'
    #             )
    #         if len(pts2d) == 8:
    #             for i in range(4, 8):  # second tag or depth layer
    #                 ax.plot(
    #                     [pts2d[i][0], pts2d[4 + (i+1)%4][0]],
    #                     [pts2d[i][1], pts2d[4 + (i+1)%4][1]],
    #                     'g--'
    #                 )
    #             # Connect verticals between two tag layers
    #             for i in range(4):
    #                 ax.plot(
    #                     [pts2d[i][0], pts2d[i + 4][0]],
    #                     [pts2d[i][1], pts2d[i + 4][1]],
    #                     'k--'
    #                 )

    # plt.grid(True)
    # plt.tight_layout()
    # plt.savefig("./april_tag_projection_plot.png", dpi=300)
    # plt.show()

    H_cam1_to_tag = np.eye(4)
    detection = best_match[0][0] # Pose of tag in camera frame
    H_cam1_to_tag[:3, :3] = detection.pose_R
    H_cam1_to_tag[:3, 3] = detection.pose_t.flatten()
    DETECTED_ID = str(detection.tag_id)
    print(f" Detected tag_id {DETECTED_ID}")
    H_world_to_tag = np.array(apriltag_world_locations[DETECTED_ID]) # Get the world frame location of the center of the tag
    # H_world_to_cam1 = inv(H_cam1_to_tag) @ H_world_to_tag
    H_world_to_cam1_prior = H_world_to_tag @ np.linalg.inv(H_cam1_to_tag) # Use the AprilTag detection as a prior for Levenberg Marquardt in cv2.solvePnP

    r_prior = np.ascontiguousarray(H_world_to_cam1_prior[:3, :3], dtype=np.float64)
    t_prior = np.ascontiguousarray(H_world_to_cam1_prior[:3, 3].reshape(3, 1), dtype=np.float64)
    rprior_rodriguez, _ = cv2.Rodrigues(r_prior)
    success, r_world_to_cam1, t_world_to_cam1 = cv2.solvePnP(worldPoints, imagePoints, 
                                                             CAM1_INTRINSICS_MAT, CAM1_DISTORTION_VEC,
                                                             rvec=rprior_rodriguez,
                                                             tvec=t_prior,
                                                             flags=cv2.SOLVEPNP_ITERATIVE)
    print("SolvePnP success? "+str(success))

    # I think this might actually be t_cam1_to_world lol

    # Returns transform from object frame -> camera frame
    # I set my object frame to be the world frame, by first transforming all object points to the world frame.

 
    print(r_world_to_cam1)
    print(t_world_to_cam1)
    R, _ = cv2.Rodrigues(r_world_to_cam1)
    H_world_to_cam1 = np.eye(4)
    H_world_to_cam1[:3,:3] = R.T # Source: https://github.com/elenagiraldo3/april_tags_autolocalization/blob/main/detect_apriltag.py#L40
    H_world_to_cam1[:3,3] = np.matmul(-R.T, t_world_to_cam1.flatten()).reshape(3)
    # H_world_to_cam1[:3,3] = t_world_to_cam1.flatten()

    pose_slam = slam_quat_to_HTM(best_match[2])
    H_sorigin_to_sbody = pose_slam


    H_cam1_to_sbody = np.linalg.inv(np.array(calibration['cam0']['T_cam_imu'])) # Calibration gives us transform from IMU to camera


    # H_world_to_sorigin = np.linalg.inv(H_sorigin_to_sbody) @ H_world_to_cam1
    # Cam1 is slam body.

    H_world_to_sorigin = H_world_to_cam1 @ np.linalg.inv(H_sorigin_to_sbody)

    Transforms.T_slam_world = H_world_to_sorigin


    world_frame_dbg = SimpleNamespace()
    origin = np.eye(4)
    world_frame_dbg.origin = origin
    world_frame_dbg.T_world_to_slam = H_world_to_sorigin
    world_frame_dbg.T_world_to_cam1 = H_world_to_cam1
    world_frame_dbg.T_world_to_cam1_prior = H_world_to_cam1_prior

    world_frame_dbg.T_world_to_tag1 = np.array(apriltag_world_locations["1"])
    world_frame_dbg.T_world_to_tag2 = np.array(apriltag_world_locations["2"])
    world_frame_dbg.T_world_to_tag3 = np.array(apriltag_world_locations["3"])
    world_frame_dbg.T_world_to_tag4 = np.array(apriltag_world_locations["4"])
    print(Transforms)

    with open(f'/home/admi3ev/ws/post/debug/world_frame_dbg.json', 'w') as fs: json.dump(vars(world_frame_dbg),fs, cls=NumpyEncoder, indent=1)

    return Transforms


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