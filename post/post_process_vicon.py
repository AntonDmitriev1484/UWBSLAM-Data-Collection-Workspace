
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
import copy

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *
from utils.vicon_parser import *


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--vicon_trial_name", type=str)
parser.add_argument("--no_orbslam", type=bool) # If we have no orbslam data, just use vicon for everything instead.
# Since my code is a mess, I'm going to do this by just aliasing the vicon data into the slam arrays.

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


args = parser.parse_args()

outpath = f'./out/{args.trial_name}_post'
if args.alias is not None: outpath = f'./out/{args.alias}_post'

out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_ml = f'{outpath}/ml'
out_synthetic = outpath+"/synthetic"
out_world = f'../world/{args.trial_name}' # Vicon can define apriltags and anchors set up in world frame

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_ml, exist_ok=True)
os.makedirs(out_synthetic, exist_ok=True)
os.makedirs(out_world, exist_ok=True)

in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_kalibr = f"../kalibr/camimu_out/{args.calibration_file}-camchain-imucam.yaml"
in_vicon = f"../vicon/out/{args.trial_name}.csv"
in_apriltags = f"../world/{args.apriltags_file}"
in_anchors = f"../world/{args.anchors_file}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

headset_data, anchor_data = parse_vicon(in_vicon) # TODO: Write a parsing function for the vicon files
# headset_data contains the pose of the marker I had on the decawave antenna in the world frame.


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

print(f" Vicon frequency { headset_data.shape[0] / (END-START)}")
f_vicon = 100
headset_data = adjust_vicon_timestamps(headset_data, START, END, f_vicon)
# Need to adjust vicon data to actual timestamps instead of just frame indices

def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


### Define all coordinate transforms
Transforms = SimpleNamespace()

# Transform from vicon marker on helmet, to center of RS camera (body frame)
Transforms.T_vmark_to_body = np.array([])

#Transform from vicon marker on anchor, to the center of the DW1000 UWB chip
Transforms.T_vmark_to_anchor = np.array([])

#Transform from vicon marker to the center of an Apriltag
Transforms.T_vmark_to_tag = np.array([])

T_world_to_anchormarker = np.eye(4)
T_world_to_anchormarker[:3, 3] = anchor_data[0, 1:4]
Transforms.T_world_to_anchor = T_world_to_anchormarker

# Transforms.T_world_to_anchor = world_to_anchor_marker[:3, 3]

Transforms.T_body_to_imu = np.array([
                                        [1, 0, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 0, 1]
                                    ])

Transforms.T_body_to_decawave = np.eye(4)
# Transforms.T_body_to_decawave[:3,3] = np.array([-0.045, -0.15, -0.025]) # For uwb_calibration_trans
Transforms.T_body_to_decawave[:3,3] = np.array([-0.12, 0.015, -0.1])

infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]

if not args.no_orbslam:
    slam_kf_data = np.loadtxt(in_slam_kf)
    slam_kf_data[:,0] *= 1e-9
    slam_data = np.loadtxt(in_slam)
    slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'
    ZERO_TIMESTAMP = slam_data[0][0]

    Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)
    # Transforms = extract_apriltag_pose_PnP(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

    if args.override_april_start is not None:
        Transforms.T_slam_world[:3, 3] = np.array(json.loads(args.override_april_start))


    # T_world_to_body = T_body_to_imu^-1 x T_imu_to_sbody^-1 x T_sorigin_to_sbody x T_world_to_sorigin
    def get_T_world_to_body(T_sorigin_to_sbody): # A function because I re-use this a lot
        T_world_to_body = (
                        Transforms.T_world_to_sorigin 
                        @ T_sorigin_to_sbody 
                        @ np.linalg.inv(Transforms.T_imu_to_sbody) 
                        @ np.linalg.inv(Transforms.T_body_to_imu)
        )
        return T_world_to_body

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

    ### Write SLAM camera trajectory
    body_poses_world_frame = [] # More accurately, this is a list of T_world_to_body
    slam_poses_slam_frame = [] # This is a list of T_sorigin_to_sbody
    slam_pose_counter = 0

    all_data_synthetic = [] # Keep interpolated points in a separate file from all.json


    dataset_slam_pose_frequency = 20 # Need something evenly divisible
    if (args.synthetic_slam_frequency > dataset_slam_pose_frequency):
        print("Error: can't be doin that buddy")
        exit()

    ### Synthetic data generation happens alongside all of the regular world frame transforms
    # Should really separate this code out...
    n_slam_skip = 0
    if args.synthetic_slam_frequency > 0: n_slam_skip = int(dataset_slam_pose_frequency/args.synthetic_slam_frequency)
    if args.synthetic_uwb_frequency > dataset_slam_pose_frequency:
        n_points = int(args.synthetic_uwb_frequency / dataset_slam_pose_frequency) # How much we need to interpolate between existing orbslam points to get this frequency of UWB
        n_skip = 1
    else:
        n_points = 1
        n_skip = 0
        if args.synthetic_uwb_frequency > 0:
            n_skip = int(dataset_slam_pose_frequency / args.synthetic_uwb_frequency)
        # ex. with 20 hz GT, and we want to simulate 5Hz UWB, we only interpolate synthetic UWB between every 4th pose pair.

    print(f"{n_points=} {n_skip=} {n_slam_skip=}")

    if args.interpolate_slam > 0: print(f"Interpolating SLAM trajectory to {args.interpolate_slam=} .")

    for i in range(slam_data.shape[0]-1):

        T_sorigin_to_sbody = slam_quat_to_HTM(slam_data[i,:])
        slam_poses_slam_frame.append( [slam_data[i,0]] + list(T_sorigin_to_sbody.flatten()) )

        T_world_to_body = get_T_world_to_body(T_sorigin_to_sbody)

        body_poses_world_frame.append( [slam_data[i,0]] + list(T_world_to_body.flatten()) )

        # NOTE: Not changing these field names because I don't want to blow up all downstream programs
        j = {
            "t": slam_data[i,0],
            "type": "slam_pose",
            "T_body_slam" : T_sorigin_to_sbody,
            "T_body_world" : T_world_to_body
        }
        all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections
        slam_pose_counter += 1

        if n_slam_skip > 0 and (slam_pose_counter % n_slam_skip == 0): all_data_synthetic.append(j)

        if n_skip > 0 and (slam_pose_counter % n_skip == 0) and n_points > 0:
            # All in the slam frame first
            current_timestamp = slam_data[i, 0]
            current_pose = T_sorigin_to_sbody
            next_pose = slam_quat_to_HTM(slam_data[i+1,:])

            dTranslation = (next_pose[:3,3] - current_pose[:3,3]) / (n_points+1)
            dt = (slam_data[i+1,0] - slam_data[i, 0]) / (n_points+1)

            Rotato = next_pose[:3, :3]

            # Use Slerp to interpolate on SE(3) rotations
            interp_interval = [slam_data[i,0], slam_data[i+1, 0]]
            interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
            slurpy = Slerp(interp_interval, interp_rots)
            interp_timestamps = np.linspace(slam_data[i,0], slam_data[i+1, 0], n_points)
            interpolated_rotations = slurpy(interp_timestamps)

            # Use kinematics to interpolate on R3 positions
            for p in range(1, n_points+1): # I think theres also a SCIPY function to do this cleaner, like SLERP but for XYZ.

                interp_slam_pose = np.eye(4)
                interp_slam_pose[:3, 3] = current_pose[:3, 3] + (dTranslation * p)
                # interp_slam_pose[:3, :3] = Rotation
                interp_slam_pose[:3,:3] = interpolated_rotations[p-1].as_matrix() # T_sorigin_to_sbody

                interp_world_pose = get_T_world_to_body(interp_slam_pose)

                interp_timestamp = current_timestamp + (p * dt)

                j = { # Note: Only going to interpolate into all.json because I just need this in the tracker.
                    "t": interp_timestamp,
                    "type": "synthetic_uwb",
                    "T_body_slam" : interp_slam_pose,
                    "T_body_world" : interp_world_pose
                }
                all_data_synthetic.append(j)

    # If we're using real UWB ranges, but have no compass
    # We interpolate on SLAM poses to match a synthetic orientation to that UWB range
    if (args.real_uwb_orientation_support):

        N_POINTS = 100
        all_uwb_mes = topic_to_processing['/uwb_ranges'][1]
        swf = np.array(body_poses_world_frame)
        for u in all_uwb_mes:

            tdiffs = np.abs(swf[:,0] - u["t"])
            slam_idx1 = np.argmin(tdiffs)
            tdiffs[slam_idx1] = np.inf
            slam_idx2 = np.argmin(tdiffs)

            istart, iend = sorted([slam_idx1, slam_idx2]) # Make sure indices are ascending

            # Make sure poses we're interpolating between are for the body in the world frame
            current_pose = get_T_world_to_body(slam_quat_to_HTM(slam_data[istart, :]))
            next_pose = get_T_world_to_body(slam_quat_to_HTM(slam_data[iend, :]))


            # Now interpolate between these two poses
            interp_interval = [slam_data[istart,0], slam_data[iend, 0]]
            interp_timestamps = np.linspace(slam_data[istart,0], slam_data[iend, 0], N_POINTS)

            # Use Slerp to interpolate on SO(3) rotations
            interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
            slurpy = Slerp(interp_interval, interp_rots)
            interpolated_rotations = slurpy(interp_timestamps)

            # Use linspace to interpolate on R3 positions
            interpolated_positions = np.linspace(current_pose[:3, 3], next_pose[:3, 3], N_POINTS)

            # Fetch the closest interpolation timestamp to the uwb measurement, and map that interpolated pose to the measurement
            idx_match = np.argmin(np.abs(interp_timestamps - u["t"]))

            world_frame_pose = np.eye(4)
            world_frame_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
            world_frame_pose[:3, 3] = interpolated_positions[idx_match]

            u2 = copy.deepcopy(u)
            u2["type"] = "assisted_uwb"
            u2["T_body_world"] = world_frame_pose
            all_data.append(u2)

        # Now, in addition to the interpolated SLAM pose, add an interpolated VICON pose to each assisted_uwb measurement.
        for mes in [a for a in all_data if a["type"]=="assisted_uwb"]:

            tdiffs = np.abs(swf[:,0] - u["t"])
            vicon_idx1 = np.argmin(tdiffs)
            tdiffs[vicon_idx1] = np.inf
            vicon_idx2 = np.argmin(tdiffs)

            istart, iend = sorted([vicon_idx1, vicon_idx2]) # Make sure indices are ascending

            # Need to get an HTM, but don't need to convert to body in world frame, because we're already in the body frame.
            current_pose = slam_quat_to_HTM(headset_data[istart, :])
            next_pose = slam_quat_to_HTM(headset_data[iend, :])


            # Now interpolate between these two poses
            interp_interval = [headset_data[istart,0], headset_data[iend, 0]]
            interp_timestamps = np.linspace(headset_data[istart,0], headset_data[iend, 0], N_POINTS)

            # Use Slerp to interpolate on SO(3) rotations
            interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
            slurpy = Slerp(interp_interval, interp_rots)
            interpolated_rotations = slurpy(interp_timestamps)

            # Use linspace to interpolate on R3 positions
            interpolated_positions = np.linspace(current_pose[:3, 3], next_pose[:3, 3], N_POINTS)

            # Fetch the closest interpolation timestamp to the uwb measurement, and map that interpolated pose to the measurement
            idx_match = np.argmin(np.abs(interp_timestamps - u["t"]))

            world_frame_pose = np.eye(4)
            world_frame_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
            world_frame_pose[:3, 3] = interpolated_positions[idx_match]

            mes["vicon_T_body_world"] = world_frame_pose
            u2 = copy.deepcopy(u)
            u2["type"] = "assisted_uwb"
            u2["T_body_world"] = world_frame_pose
            all_data.append(u2)

    # Compute velocities from body poses in the world frame
    # This way you can set a velocity prior at any time

    np_body_poses_world_frame = np.array(body_poses_world_frame)
    dt = np.diff(np_body_poses_world_frame[:,0])
    dx = np.diff(np_body_poses_world_frame[:,4]) / dt
    dy = np.diff(np_body_poses_world_frame[:,7]) / dt
    dz = np.diff(np_body_poses_world_frame[:,10]) / dt
    headset_data_velocity_world_frame = np.vstack((np_body_poses_world_frame[:np_body_poses_world_frame.shape[0]-1,0], dx, dy, dz)).T
    # By default. I map the velocity between t and t+1 to timestamp t. This should be good enough for prioring.
    # These are of dubious quality.... lol

    print(f" SLAM world 0: {body_poses_world_frame[0][0]} SLAM velocity world 0: {headset_data_velocity_world_frame[0][0]}")

    # For all slam poses
    slam_idx = 0
    for i, mes in enumerate(all_data):
        if mes["type"] == "slam_pose":
            mes_ = mes
            if slam_idx < headset_data_velocity_world_frame.shape[0]:
                mes_["v_world"] = {
                    "vx": headset_data_velocity_world_frame[ slam_idx, 1],
                    "vy": headset_data_velocity_world_frame[ slam_idx, 2],
                    "vz": headset_data_velocity_world_frame[ slam_idx, 3]
                }
            all_data[i] = mes_ # Extend each pose to also include its computed velocity
            slam_idx +=1






    with open(f'{out_ml}/body_poses_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(body_poses_world_frame))
    with open(f'{out_ml}/slam_poses_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_poses_slam_frame))

    body_poses_world_frame_tum = []
    with open(f'{outpath}/body_poses_world_frame_tum.txt', 'w') as fs: csv.writer(fs, delimiter=' ').writerows(filtt2(body_poses_world_frame_tum))

    ### Write SLAM KF trajectory

    kf_body_poses_world_frame = []
    kf_slam_poses_slam_frame = []
    for i in range(slam_kf_data.shape[0]):

        T_body_slam = slam_quat_to_HTM(slam_kf_data[i,:])
        kf_slam_poses_slam_frame.append( [slam_kf_data[i,0]] + list(T_body_slam.flatten()) )

        T_body_world = Transforms.T_slam_world @ T_body_slam

        kf_body_poses_world_frame.append( [slam_kf_data[i,0]] + list(T_body_slam.flatten()))

        j = {
            "t": slam_kf_data[i,0],
            "type": "slam_kf_pose",
            "T_body_slam" : T_body_slam,
            "T_body_world" : T_body_world
        }
        all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections


    with open(f'{out_ml}/kf_body_poses_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(kf_body_poses_world_frame))
    with open(f'{out_ml}/kf_slam_poses_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(kf_slam_poses_slam_frame))


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



    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if hasattr(obj, '__dict__'):
                return vars(obj)
            return super().default(obj)



    ### Copy all world information: transforms, anchors, apriltags, to output
    shutil.copy(in_anchors, f'{outpath}/anchors.json')
    shutil.copy(in_apriltags, f'{outpath}/apriltags.json')
    with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


    # Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
    print("Checking frequency of real data")
    print(f" Measured UWB frequency {uwb_message_count / (END-START)}")
    print(f" Measured SLAM frequency {len(slam_data) / (END-START)}")

    print("Checking frequency of synthetic data")

    nuwb, ngt = (0,0)
    for mes in all_data_synthetic:
        if mes["type"] == "synthetic_uwb": nuwb+=1
        if mes["type"] == "slam_pose": ngt+=1

    generated_fuwb = nuwb / (END-START)
    generated_fgt = ngt / (END-START)

    if args.crop_start is not None: # These numbers might be a bit off with crop start
        generated_fuwb = nuwb / (END-args.crop_start)
        generated_fgt = ngt / (END-args.crop_start)

    print(f" UWB requested f={args.synthetic_uwb_frequency} , generated f={generated_fuwb}")
    print(f" GT requested f={args.synthetic_slam_frequency} , generated f={generated_fgt}")

    # Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
    all_data = filtt(all_data)
    all_data = sorted(all_data, key=lambda x: x["t"])
    json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

    # All data syntehtic is (all real data except slam) + (real SLAM (filtered) + synthetic UWB (created from interpolating on real SLAM))
    all_data_synthetic = filtt( [a for a in all_data if not a["type"] == "slam_pose"] + all_data_synthetic) 
    all_data_synthetic = sorted(all_data_synthetic, key=lambda x: x["t"])

    json.dump(all_data_synthetic, open(outpath+"/synthetic"+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}.json", 'w'), cls=NumpyEncoder, indent=1)
    # So all synthetic files will have a unique name


    json.dump(args.__dict__, open(out_synthetic+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}_meta.json", 'w'), cls=NumpyEncoder, indent=1)
else: # No ORBSLAM available

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

    ### Write SLAM camera trajectory
    body_poses_world_frame = [] # More accurately, this is a list of T_world_to_body
    all_data_synthetic = [] # Keep interpolated points in a separate file from all.json

    # TODO: Write vicon poses to all_json and body_poses_world_frame
    # let headset_data be formatted like slam_data, i.e. TUM and timestamp
    # then I can convert to HTM

    for i in range(headset_data.shape[0]-1):

        # T_world_to_body = get_T_world_to_body(T_sorigin_to_sbody) # SO I just need to work out the coord frame here
        T_world_to_body = slam_quat_to_HTM(headset_data[i,:]) # Convert headset data from quat to HTM
        body_poses_world_frame.append( [headset_data[i,0]] + list(T_world_to_body.flatten()) )

        # NOTE: Not changing these field names because I don't want to blow up all downstream programs
        j = {
            "t": headset_data[i,0],
            "type": "vicon_pose",
            "T_body_world" : T_world_to_body
        }
        all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections



    # If we're using real UWB ranges, but have no compass
    # We interpolate on SLAM poses to match a synthetic orientation to that UWB range
    if (args.real_uwb_orientation_support):

        N_POINTS = 100
        all_uwb_mes = topic_to_processing['/uwb_ranges'][1]
        vwf = np.array(body_poses_world_frame) # 'vicon world frame' instead of 'slam world frame'
        for u in all_uwb_mes:
            tdiffs = np.abs(vwf[:,0] - u["t"])
            vicon_idx1 = np.argmin(tdiffs)
            tdiffs[vicon_idx1] = np.inf
            vicon_idx2 = np.argmin(tdiffs)

            istart, iend = sorted([vicon_idx1, vicon_idx2]) # Make sure indices are ascending

            # Need to get an HTM, but don't need to convert to body in world frame, because we're already in the body frame.
            current_pose = slam_quat_to_HTM(headset_data[istart, :])
            next_pose = slam_quat_to_HTM(headset_data[iend, :])


            # Now interpolate between these two poses
            interp_interval = [headset_data[istart,0], headset_data[iend, 0]]
            interp_timestamps = np.linspace(headset_data[istart,0], headset_data[iend, 0], N_POINTS)

            # Use Slerp to interpolate on SO(3) rotations
            interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
            slurpy = Slerp(interp_interval, interp_rots)
            interpolated_rotations = slurpy(interp_timestamps)

            # Use linspace to interpolate on R3 positions
            interpolated_positions = np.linspace(current_pose[:3, 3], next_pose[:3, 3], N_POINTS)

            # Fetch the closest interpolation timestamp to the uwb measurement, and map that interpolated pose to the measurement
            idx_match = np.argmin(np.abs(interp_timestamps - u["t"]))

            world_frame_pose = np.eye(4)
            world_frame_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
            world_frame_pose[:3, 3] = interpolated_positions[idx_match]

            u2 = copy.deepcopy(u)
            u2["type"] = "assisted_uwb"
            u2["T_body_world"] = world_frame_pose
            all_data.append(u2)



    # Compute velocities from body poses in the world frame
    # This way you can set a velocity prior at any time

    np_body_poses_world_frame = np.array(body_poses_world_frame)
    dt = np.diff(np_body_poses_world_frame[:,0])
    dx = np.diff(np_body_poses_world_frame[:,4]) / dt
    dy = np.diff(np_body_poses_world_frame[:,7]) / dt
    dz = np.diff(np_body_poses_world_frame[:,10]) / dt
    headset_data_velocity_world_frame = np.vstack((np_body_poses_world_frame[:np_body_poses_world_frame.shape[0]-1,0], dx, dy, dz)).T

    print(f" SLAM world 0: {body_poses_world_frame[0][0]} SLAM velocity world 0: {headset_data_velocity_world_frame[0][0]}")

    # For all slam poses
    slam_idx = 0
    for i, mes in enumerate(all_data):
        if mes["type"] == "slam_pose":
            mes_ = mes
            if slam_idx < headset_data_velocity_world_frame.shape[0]:
                mes_["v_world"] = {
                    "vx": headset_data_velocity_world_frame[ slam_idx, 1],
                    "vy": headset_data_velocity_world_frame[ slam_idx, 2],
                    "vz": headset_data_velocity_world_frame[ slam_idx, 3]
                }
            all_data[i] = mes_ # Extend each pose to also include its computed velocity
            slam_idx +=1




    with open(f'{out_ml}/body_poses_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(body_poses_world_frame))

    body_poses_world_frame_tum = []
    with open(f'{outpath}/body_poses_world_frame_tum.txt', 'w') as fs: csv.writer(fs, delimiter=' ').writerows(filtt2(body_poses_world_frame_tum))

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



    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if hasattr(obj, '__dict__'):
                return vars(obj)
            return super().default(obj)



    ### Copy all world information: transforms, anchors, apriltags, to output

    # Use Decawave and AprilTag marker information to compute and
    out_anchors = open(f'{out_world}/anchors_{args.trial_name}.json', 'w')
    world_frame_anchors = [{
        "ID":3,
        "position": Transforms.T_world_to_anchor[:3, 3]
    }]
    json.dump(world_frame_anchors, out_anchors, cls=NumpyEncoder, indent=1)
    out_anchors_trial = open(f'{outpath}/anchors.json', 'w')
    json.dump(world_frame_anchors, out_anchors_trial, cls=NumpyEncoder, indent=1)


    out_tags = open(f'{out_world}/apriltags_{args.trial_name}.json', 'w')
    # TODO: 

    world_frame_tags = {
        "1": None
    }
    json.dump(world_frame_tags, out_tags, cls=NumpyEncoder, indent=1)
    out_tags_trial = open(f'{outpath}/apriltags.json', 'w')
    json.dump(world_frame_tags, out_tags_trial, cls=NumpyEncoder, indent=1)


    with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)

    # Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
    print("Checking frequency of real data")
    print(f" Measured UWB frequency {uwb_message_count / (END-START)}")
    print(f" Measured vicon frequency {headset_data.shape[0] / (END-START)}")

    # Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
    all_data = filtt(all_data)
    all_data = sorted(all_data, key=lambda x: x["t"])
    json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

    # All data syntehtic is (all real data except slam) + (real SLAM (filtered) + synthetic UWB (created from interpolating on real SLAM))
    all_data_synthetic = filtt( [a for a in all_data if not a["type"] == "slam_pose"] + all_data_synthetic) 
    all_data_synthetic = sorted(all_data_synthetic, key=lambda x: x["t"])

    json.dump(all_data_synthetic, open(outpath+"/synthetic"+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}.json", 'w'), cls=NumpyEncoder, indent=1)
    # So all synthetic files will have a unique name


    json.dump(args.__dict__, open(out_synthetic+f"/all_synthetic_{args.synthetic_slam_frequency}_{args.synthetic_uwb_frequency}_meta.json", 'w'), cls=NumpyEncoder, indent=1)
