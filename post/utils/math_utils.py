

import numpy as np
from scipy.spatial.transform import Rotation as R

def slam_quat_to_HTM(nparr):
    translation = nparr[1:4]
    quat = nparr[4:8]
    
    r = R.from_quat(quat)
    rotation_matrix = r.as_matrix()

    # Assemble homogeneous transformation matrix (4x4)
    H = np.eye(4)
    H[:3, :3] = rotation_matrix
    H[:3, 3] = translation

    return H

def slam_HTM_to_TUM(nparr):
    if len(nparr) != 17:
        print(nparr)
        raise ValueError("Expected 17 elements: [timestamp, 16 HTM elements]")

    timestamp = nparr[0]
    T_flat = nparr[1:]
    T = np.array(T_flat).reshape((4, 4))

    # Extract translation and rotation
    t = T[:3, 3]
    R_mat = T[:3, :3]
    quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

    return [timestamp, t[0], t[1], t[2], quat[0], quat[1], quat[2], quat[3]]