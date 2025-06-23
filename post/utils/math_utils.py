

import numpy as np
from scipy.spatial.transform import Rotation as R

def slam_quat_to_HTM(nparr):
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