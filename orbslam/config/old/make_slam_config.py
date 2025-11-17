import numpy as np
import argparse
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('-cam_data', '-c' )
args = parser.parse_args()

# First part of this path depends on whether you're in docker or on base
IMU_FILE = "/home/admi3ev/kalibr_mount/allan_variance_out/imu.yaml" # TODO: Change this per each camera you calibrate
CAM_IMU_FILE = "/home/admi3ev/kalibr_mount/kalibr_cam_imu_calibration_out/cam_target_slow-camchain-imucam.yaml"

CONFIG_TEMPLATE = "/home/admi3ev/kalibr_mount/orb_slam3_configs/rs_slam_config_ex.yaml"

OUT_CONFIG = "/home/admi3ev/kalibr_mount/orb_slam3_configs/rs_slam_config.yaml"


with open(IMU_FILE, 'r') as file:
    imu_yaml = yaml.safe_load(file)

with open(CAM_IMU_FILE, 'r') as file:
    camimu_yaml = yaml.safe_load(file)

with open(CONFIG_TEMPLATE, 'r') as file:
    temp_yaml = yaml.safe_load(file)


camimu_yaml = camimu_yaml['cam0']
print(imu_yaml)
print(camimu_yaml)

print(temp_yaml)

temp_yaml['Camera1.fx'] = camimu_yaml['intrinsics'][0]
temp_yaml['Camera1.fy'] = camimu_yaml['intrinsics'][1]
temp_yaml['Camera1.cx'] = camimu_yaml['intrinsics'][2]
temp_yaml['Camera1.cy'] = camimu_yaml['intrinsics'][3]

temp_yaml['Camera1.k1'] = camimu_yaml['distortion_coeffs'][0]
temp_yaml['Camera1.k2'] = camimu_yaml['distortion_coeffs'][1]
temp_yaml['Camera1.p1'] = camimu_yaml['distortion_coeffs'][2]
temp_yaml['Camera1.p2'] = camimu_yaml['distortion_coeffs'][3]

temp_yaml['IMU.NoiseGyro'] = imu_yaml['gyroscope_noise_density']
temp_yaml['IMU.NoiseAcc'] = imu_yaml['accelerometer_noise_density']
temp_yaml['IMU.GyroWalk'] = imu_yaml['gyroscope_random_walk']
temp_yaml['IMU.AccWalk'] = imu_yaml['accelerometer_random_walk']
temp_yaml['IMU.Frequency'] = imu_yaml['update_rate']

print(np.array(camimu_yaml['T_cam_imu']))

arr = np.linalg.inv(np.array(camimu_yaml['T_cam_imu'])).tolist()
temp_yaml['IMU.T_b_c1']['data'] = arr


with open(OUT_CONFIG, 'w') as outfile:
    yaml.dump(temp_yaml, outfile, default_flow_style=None)

print(temp_yaml)