import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_tum(arr, degrees=True):
    """
    Convert Euler + translation to TUM format string.
    
    Args:
        t: timestamp (float)
        rx, ry, rz: rotation angles (Euler) [degrees if degrees=True]
        tx, ty, tz: translation
        degrees: True if angles are in degrees
    Returns:
        str: formatted TUM line
    """
    rot = R.from_euler('xyz', arr[1:4], degrees=degrees)
    qx, qy, qz, qw = rot.as_quat()  # returns [x, y, z, w]
    return np.array([arr[0], arr[4], arr[5], arr[6], qx, qy, qz, qw])


def parse_vicon(file):
    # Read the file, skipping first 3 rows (metadata headers + units)
    # data = np.loadtxt(file, delimiter=",", skiprows=5)

    # Because god forbid this be straightforward
    rows = []
    frame_counter = 1
    line_counter = 0
    with open(file, 'r') as f:
        for line in f:
            if line_counter >= 5:
                # Strip newline, split by comma, convert to float
                row = [x for x in line.strip().split(',')]
                if '' in row: # Vicon just doesn't include pose for some reason
                    row = rows[len(rows)-1] # set it to the last tracked pose
                    row[0] = frame_counter
                row = [float(x) for x in row]
                rows.append(row)
                frame_counter += 1
            line_counter += 1

    data = np.array(rows)

    # Ex. for first object, Frame (0) + R(rad) (2-4) + t(mm) (5-7)
    headset_arr_rpy = data[:, [0, 2, 3, 4, 5, 6, 7]]
    anchor_arr_rpy = data[:, [0, 8, 9, 10, 11, 12, 13]]

    headset_arr = np.array([euler_to_tum(row) for row in headset_arr_rpy])
    anchor_arr = np.array([euler_to_tum(row) for row in anchor_arr_rpy])


    # Scale to m
    print(headset_arr[0, :])
    headset_arr[:, 1:4] /= 1000
    anchor_arr[:, 1:4] /= 1000
    print(headset_arr[0, :])
    return headset_arr, anchor_arr

def adjust_vicon_timestamps(arr, ros_start, ros_end, f_vicon):
    # How fast does vicon capture frames?
    # Can just calculate this by hand.

    vicon_start = ros_start
    vicon_end = ros_start + (arr.shape[0] / f_vicon) # N frames / N fps = total vicon recording time.
    timestamps = np.linspace(vicon_start, vicon_end, arr.shape[0]) 
    # assuming frames are evenly spaced, interpolate over total time interval

    arr[:,0] = timestamps

    return arr


