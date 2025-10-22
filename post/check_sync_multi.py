import json
import numpy as np
import matplotlib.pyplot as plt

fs= open('/home/antond2/ws/post/out/irl4_walk_together2_post/all.json', 'r')

data = json.load(fs)


# timestamps = {"2": [], "3": [], "4": []}
timestamps = { "3": [], "4": []}
for mes in data:
    timestamps[mes["src"]].append(mes["t"])

# Plot
plt.figure(figsize=(10, 4))

for id, tstps in timestamps.items():
    plt.scatter(tstps, [id]*len(tstps), color='blue', label=f"Nuc {id} timestamps")
    print(f"Nuc {id} elapsed time {tstps[-1]-tstps[0]} timestamps")

plt.xlabel("Timestamp")
# plt.yticks([])  # Hide y-axis
plt.title("Timestamp scatter")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()