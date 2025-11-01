import cv2
import numpy as np
from utils.math_utils import *
import copy

def proc_range(msg, arr_ref): #TODO update this to new UWB topic
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

    arr_ref.append(j)

    # csv_row = []
    # for k, v in j.items(): csv_row.append(v)
    # arr_ref.append(csv_row)

    return j

# def proc_rgb_frame(msg, arr_ref):
#     #rgb8 encoding

#     timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
#     encoding = msg.encoding
#     arr = msg.data

#     # Make new file in out_rgb, labeled with timestamp.
#     img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width, 3))
#     name = str(timestamp)+".png"
#     cv2.imwrite(out_rgb+"/"+name, cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)) # Not exactly sure what cvtColor does...

#     return {"t":timestamp, "type":"rgb", "name":name}

# def proc_depth_frame(msg, arr_ref):
#     timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
#     encoding = msg.encoding
#     arr = msg.data

#     img_np = np.frombuffer(arr, dtype=np.uint16).reshape((msg.height, msg.width)) # Output says unit8 but encoding says 16UC1
#     name = str(timestamp)+".png"
#     cv2.imwrite(out_depth+"/"+name, img_np)

#     return {"t":timestamp, "type":"depth", "name":name}

def proc_infra1_frame(msg, arr_ref):
    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    name = str(timestamp) +".png"
    img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width)) # Output says unit8 but encoding says 16UC1

    j = {"t":timestamp, "type":"infra1", "name":name, "raw": img_np}
    arr_ref.append(j)

    return j

def proc_infra2_frame(msg, arr_ref):
    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data
    name = str(timestamp) +".png"
    img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width)) # Output says unit8 but encoding says 16UC1
    
    j = {"t":timestamp, "type":"infra2", "name":name, "raw": img_np}
    arr_ref.append(j)

    return j

# I set it to unify accel and gyro, does unified accel and gyro go to the accel topic?
def proc_imu(msg, arr_ref):

    # I should be looking at a topic called 'imu' -> Interesting, I think I forgot to listen to this topic.
    # Despite unite_imu being set to 2, there is no 'imu' topic available in the ros2 topics list

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    j = {"t":timestamp, "type":"imu", "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z, 
            "gx":msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz":msg.angular_velocity.z}

    arr_ref.append(j)

    # csv_row = []
    # for k, v in j.items(): csv_row.append(v)
    # arr_ref.append(csv_row)

    return j


def aggregate_uwb(topic_to_processing, uwb_csv, uwb_range_distribution):
    for j in topic_to_processing['/uwb_ranges'][1]:
        csv_row = []
        for k, v in j.items():
            csv_row.append(v) # This should iterate in the order of how keys are originally defined in the json
        uwb_csv.append(csv_row)
        uwb_range_distribution.append(j['range'])

    return topic_to_processing['/uwb_ranges'][1]

def aggregate_imu(topic_to_processing, imu_csv):
    for j in topic_to_processing['/camera/camera/imu'][1]:
        csv_row = []
        for k, v in j.items():
            if k != "type":
                csv_row.append(v)
        imu_csv.append(csv_row)
#     k='t' v=1761788482.5987916
# k='ax' v=-9.739026069641113
# k='ay' v=0.6378458738327026
# k='az' v=-0.24674910306930542
# k='gx' v=-0.000980242039076984
# k='gy' v=-0.005792018957436085
# k='gz' v=0.0010896283201873302
# csv_row=[1761788482.5987916, -9.739026069641113, 0.6378458738327026, -0.24674910306930542, -0.000980242039076984, -0.005792018957436085, 0.0010896283201873302]
    return topic_to_processing['/camera/camera/imu'][1]

def aggregate_infra1(topic_to_processing, out_infra1):
    data = []
    for j in topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]:
        cv2.imwrite(out_infra1+"/"+j["name"], j["raw"])
        j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
        data.append(j_no_image)
    return data

def aggregate_infra2(topic_to_processing, out_infra1):
    data = []
    for j in topic_to_processing['/camera/camera/infra2/image_rect_raw'][1]:
        cv2.imwrite(out_infra1+"/"+j["name"], j["raw"])
        j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
        data.append(j_no_image)
    return data

def aggregate_tracker(trackbody_to_mybody_func, tracker_data_tum):
    tracker_body_poses = []  # All T_world_to_body as flattened HTM

    for i in range(tracker_data_tum.shape[0]-1):
        tracker_pose = slam_quat_to_HTM(tracker_data_tum[i, :])   # tracker pose as HTM
        T_world_to_body = trackbody_to_mybody_func(tracker_pose)     # convert tracker to body frame
        tracker_body_poses.append([tracker_data_tum[i, 0]] + list(T_world_to_body.flatten()))
    
    tracker_body_poses = np.array(tracker_body_poses)

    # Compute velocity for body (using translation columns of HTM)
    dt = np.diff(tracker_body_poses[:, 0])
    dx = np.diff(tracker_body_poses[:, 4]) / dt
    dy = np.diff(tracker_body_poses[:, 7]) / dt
    dz = np.diff(tracker_body_poses[:, 10]) / dt
    tracker_body_velocities = np.vstack((
        tracker_body_poses[:-1, 0],  # timestamps
        dx, dy, dz
    )).T

    return tracker_body_poses, tracker_body_velocities

def aggregate_assisted_uwb(uwb_json, trackbody_to_mybody_func, tracker_data_tum, N_POINTS):
    assisted_uwb_json = []

    # Still using 'vicon' syntax but thats ok who cares
    # Must interpolate in original frame, then apply your transform: hence trackbody_to_mybody_func
    
    vwf = tracker_data_tum # 'vicon world frame' instead of 'slam world frame'
    
    for u in uwb_json:
        # Find the closest timestamp vicon measurements to our UWB range
        tdiffs = np.abs(vwf[:,0] - u["t"])
        vicon_idx1 = np.argmin(tdiffs)
        tdiffs[vicon_idx1] = np.inf
        vicon_idx2 = np.argmin(tdiffs)
        istart, iend = sorted([vicon_idx1, vicon_idx2]) # Make sure indices are ascending

        start_pose = slam_quat_to_HTM(vwf[istart, :]) # Convert quat -> HTM
        end_pose = slam_quat_to_HTM(vwf[iend, :])

        interp_pose = interpolate_pose(
            start_pose, vwf[istart, 0],
            end_pose, vwf[iend, 0],
            u["t"], N_POINTS
        ) # Interpolated pose is a T_world_to_cam1

        interp_pose_world = trackbody_to_mybody_func(interp_pose)

        u2 = copy.deepcopy(u)
        u2["type"] = "assisted_uwb"
        u2["T_body_world"] = interp_pose_world
        assisted_uwb_json.append(u2)

    return assisted_uwb_json
