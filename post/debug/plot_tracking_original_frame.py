import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting

# Load CSV file
# Replace 'data.csv' with your actual filename
df = pd.read_csv("/home/admi3ev/ws/vicon/out/irl3_los_walking.csv")

# Filter only the LeftRS rows
left_rs = df[df["subject"] == "LeftRS"]

# Extract positions
x = left_rs["x"].values
y = left_rs["y"].values
z = left_rs["z"].values

# Create 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

# Plot the trajectory
ax.plot(x, y, z, label="LeftRS trajectory")

# Mark start and end points
ax.scatter(x[0], y[0], z[0], color="green", s=60, label="Start")
ax.scatter(x[-1], y[-1], z[-1], color="red", s=60, label="End")

# Labels
ax.set_xlabel("X position")
ax.set_ylabel("Y position")
ax.set_zlabel("Z position")
ax.set_title("3D Trajectory of LeftRS")
ax.legend()

plt.show()
