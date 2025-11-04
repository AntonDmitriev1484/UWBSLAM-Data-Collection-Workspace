import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import json


# def draw_axes(ax, T, manual_origin = [0,0,0], length=0.1):
#     """Draw coordinate frame axes given 4x4 transform T (inertial→imu)."""
#     origin = np.array(manual_origin)

#     T = np.linalg.inv(T)

#     x_axis = origin + T[:3, 0] * length
#     y_axis = origin + T[:3, 1] * length
#     z_axis = origin + T[:3, 2] * length

#     ax.quiver(*origin, *(x_axis - origin), color='r', label='x-axis' if not hasattr(ax, '_x_drawn') else "")
#     ax.quiver(*origin, *(y_axis - origin), color='g', label='y-axis' if not hasattr(ax, '_y_drawn') else "")
#     ax.quiver(*origin, *(z_axis - origin), color='b', label='z-axis' if not hasattr(ax, '_z_drawn') else "")

#     ax._x_drawn, ax._y_drawn, ax._z_drawn = True, True, True
def draw_axes(ax, T, manual_origin, length=0.1): 
    """Draw coordinate axes from transformation matrix T.""" 
    H = np.linalg.inv(T) 

    origin = (H @ np.array([0,0,0,1]))[:3] 
    origin = manual_origin
    x_axis = (H @ np.array([1,0,0,1]))[:3] 
    y_axis = (H @ np.array([0,1,0,1]))[:3] 
    z_axis = (H @ np.array([0,0,1,1]))[:3] 
    ax.quiver(*origin, *(x_axis), color='r') 
    ax.quiver(*origin, *(y_axis), color='g') 
    ax.quiver(*origin, *(z_axis), color='b')

if __name__ == "__main__":
    # --- Transforms & bias vectors ---

    T_ori1 = np.eye(4)
    T_ori1[:3, :3] = np.array([[ 0.03033491, -0.99953822,  0.00177115],
       [-0.01663786, -0.00227666, -0.99985899],
       [ 0.99940131,  0.03030116, -0.01669924]]) 
    b1 = np.array([-0.10481777, -0.02123918 , 0.38263766] )
    g1 = np.array([ 0.017375   ,-9.80861669 ,-0.1638195 ])
    a1 = np.array([ 0.12219277, -9.78737751, -0.54645716])


    T_ori2 = np.eye(4)
    T_ori2[:3, :3] = np.array([
        [-0.99674707, -0.02998852, 0.07480619],
        [-0.074566, -0.0090509, -0.99717501],
        [0.03058086, -0.99950926, 0.00678533]
    ])
    b2 = np.array([ 0.45318633 ,-0.00741014,  0.62170884] )
    g2 = np.array([ 0.73384872 ,-9.78228681,  0.06656412])
    a2 = np.array([ 0.28066239, -9.77487667, -0.55514472]) # Plotting wrong?

    T_ori3 = np.eye(4)
    T_ori3[:3, :3] = np.array([
        [0.0172277, 0.9972338, 0.0723046],
        [-0.99149186, 0.00770716, 0.12994032],
        [0.12902361, -0.07392799, 0.98888197]
    ])

    b3 = np.array([ 0.85730502 , 1.41135235, -0.08752875])
    g3 = np.array([0.70930808 ,1.27471451 ,9.70093217])
    a3 = np.array([-0.14799694, -0.13663784,  9.78846092])

    T_ori4 = np.eye(4)
    T_ori4[:3, :3] = np.array([
        [-9.01497005e-02,  3.27603984e-03, -9.95922838e-01],
       [-9.95404815e-01,  3.21198786e-02,  9.02084665e-02],
       [ 3.22844472e-02,  9.99478655e-01,  3.65388397e-04]])

    b4 = np.array([-0.02945025,  0.24091875,  0.22525405])
    g4 = np.array([-9.77000304e+00,  8.84945056e-01,  3.58446017e-03])
    a4 = np.array([-9.74055279,  0.64402631 ,-0.22166959])

    transforms = [T_ori1, T_ori2, T_ori3, T_ori4]
    biases = [b1, b2, b3, b4]
    gravity = [g1, g2, g3, g4]
    accel = [a1, a2, a3, a4]

   # --- Plot all frames ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.set_title("IMU Frames & Bias Vectors")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

for i, (T, b, g, a) in enumerate(zip(transforms, biases, gravity, accel)):
    origin = np.array([2*i, 0, 0])
    draw_axes(ax, T, manual_origin=origin, length=0.3)

    SCALE = 1

    R = T[:3,:3].T
    print(R)

    g_world = R @ g
    ax.quiver(*origin, *(g_world * SCALE), color='purple', linewidth=2, arrow_length_ratio=0.1)

    a_world = R @ a
    print(a)
    print(a_world)
    ax.quiver(*origin, *(a_world * SCALE), color='pink', linewidth=2, arrow_length_ratio=0.1)
    
    b_world = R @ b
    ax.quiver( *(origin + (a_world * SCALE)), *(b_world * SCALE), color='k', linewidth=2, arrow_length_ratio=0.1)
    ax.text(*origin, f"T{i+1}", color='black')

all_points = np.array([b for b in biases])
lim = np.max(np.abs(all_points)) * 1.5
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-lim, lim)
plt.tight_layout()

plt.show(block=False)


# --- Second figure ---
fig2 = plt.figure(figsize=(8, 6))
ax2 = fig2.add_subplot()
ax2.set_title("Raw Accel Values")
ax2.set_xlabel("Time")
ax2.set_ylabel("Accel")

for i in range(1, 3):
    accel = []
    calib = json.load(open(f'../out/irl5_imu_bias_ori{i}_post/calibration.json'))
    for mes in calib:
        if mes["type"] == "imu":
            accel.append([mes["t"], mes["ax"], mes["ay"], mes["az"]])
    accel = np.array(accel)

    accel[:, 0] -= accel[0, 0]

    if i == 2:
        ax2.plot(accel[:, 0], accel[:, 1], color='darkred')
        ax2.plot(accel[:, 0], accel[:, 2], color='darkgreen')
        ax2.plot(accel[:, 0], accel[:, 3], color='darkblue')
    else:
        ax2.plot(accel[:, 0], accel[:, 1], color='red')
        ax2.plot(accel[:, 0], accel[:, 2], color='green')
        ax2.plot(accel[:, 0], accel[:, 3], color='blue')

plt.tight_layout()
plt.show()
