import matplotlib.pyplot as plt
import csv
import numpy as np

def load_imu_csv(filepath):
    timestamps = []
    acc = []  # ax, ay, az
    gyro = []  # gx, gy, gz

    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) != 7:
                print(f"Skipping malformed row: {row}")
                continue
            try:
                t = float(row[0])
                ax, ay, az = float(row[1]), float(row[2]), float(row[3])
                gx, gy, gz = float(row[4]), float(row[5]), float(row[6])
            except ValueError:
                print(f"Skipping invalid row: {row}")
                continue

            timestamps.append(t)
            acc.append((ax, ay, az))
            gyro.append((gx, gy, gz))

    return timestamps, list(zip(*acc)), list(zip(*gyro))


def plot_imu(timestamps, acc, gyro):
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("IMU Data from CSV")

    print(acc)

    # Accelerometer
    axs[0].plot(timestamps, acc[0], label='ax')
    axs[0].plot(timestamps, acc[1], label='ay')
    axs[0].plot(timestamps, acc[2], label='az')
    axs[0].set_ylabel("Acceleration (m/s²)")
    axs[0].legend()
    axs[0].grid()

    # Gyroscope
    axs[1].plot(timestamps, gyro[0], label='gx')
    axs[1].plot(timestamps, gyro[1], label='gy')
    axs[1].plot(timestamps, gyro[2], label='gz')
    axs[1].set_ylabel("Angular Velocity (rad/s)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid()

    acc = np.array(acc).T

    for i in range(0,3):
        print(f" First window { np.std(acc[0:2000, i]) }")
        print(f" Second window { np.std(acc[2000:4000, i]) }")
        print(f" Third window { np.std(acc[4000:6000, i]) }")

    # 0.19 is likely too large to be a bias,
    # This averaging should be done in the navigation frame
    # GTSAM should be giving us the correct result, 
    # because it rotates into the nav frame to perform gravity compensation, and has its loss function defined there
    # should not be giving us the same as this local frame windowed average
    # Local frame windowed average should be more susecptible to coordinate frame error.

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    filepath = "./out/imu_bias_cam2_align_post/ml/imu_data.csv"  # Change t_
    timestamps, acc, gyro = load_imu_csv(filepath)
    plot_imu(timestamps, acc, gyro)
