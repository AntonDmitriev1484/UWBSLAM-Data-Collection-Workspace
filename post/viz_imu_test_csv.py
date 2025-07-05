import matplotlib.pyplot as plt
import csv

def load_imu_csv(filepath):
    timestamps = []
    acc = []  # ax, ay, az
    gyro = []  # gx, gy, gz

    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) != 8:
                print(f"Skipping malformed row: {row}")
                continue
            try:
                t = float(row[0])
                ax, ay, az = float(row[2]), float(row[3]), float(row[4])
                gx, gy, gz = float(row[5]), float(row[6]), float(row[7])
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

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    filepath = "./out/stereoi_circle2_post/ml/imu_data.csv"  # Change t_
    timestamps, acc, gyro = load_imu_csv(filepath)
    plot_imu(timestamps, acc, gyro)
