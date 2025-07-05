import json
import matplotlib.pyplot as plt

def load_imu_data(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)
    imu_data = [entry for entry in data if entry["type"] == "imu"]
    return imu_data

def plot_imu_data(imu_data):
    t = [entry["t"] for entry in imu_data]

    ax = [entry["ax"] for entry in imu_data]
    ay = [entry["ay"] for entry in imu_data]
    az = [entry["az"] for entry in imu_data]

    gx = [entry["gx"] for entry in imu_data]
    gy = [entry["gy"] for entry in imu_data]
    gz = [entry["gz"] for entry in imu_data]

    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("IMU Data Over Time")

    # Accelerometer
    axs[0].plot(t, ax, label='ax')
    axs[0].plot(t, ay, label='ay')
    axs[0].plot(t, az, label='az')
    axs[0].set_ylabel("Acceleration (m/s²)")
    axs[0].legend()
    axs[0].grid()

    # Gyroscope
    axs[1].plot(t, gx, label='gx')
    axs[1].plot(t, gy, label='gy')
    axs[1].plot(t, gz, label='gz')
    axs[1].set_ylabel("Angular Velocity (rad/s)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    imu_data = load_imu_data("./out/stereoi_circle2_post/all.json")  # Replace with your actual file path
    plot_imu_data(imu_data)
