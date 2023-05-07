import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.transforms import Affine2D
from scipy.interpolate import interp1d

# Read JSON data from file
json_file = "build/collision_in_node0.json"
with open(json_file, "r") as f:
    data = json.load(f)

track1 = data["track1"]
track2 = data["track2"]
collision_time_ms = min(data["pose1"]["time_ms"], data["pose2"]["time_ms"])

# Get positions and headings of vehicles in track1 and track2
positions1 = np.array([pose["pos"] for pose in track1])
headings1 = np.array([pose["h"] for pose in track1])
positions2 = np.array([pose["pos"] for pose in track2])
headings2 = np.array([pose["h"] for pose in track2])
times1 = np.array([pose["time_ms"] for pose in track1])
times2 = np.array([pose["time_ms"] for pose in track2])

# Create plot
fig, ax = plt.subplots(figsize=(15, 15))
ax.set_xlim([0, 250])
ax.set_ylim([0, 250])
ax.set_aspect("equal")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Plot trajectories
ax.plot(positions1[:, 0], positions1[:, 1], "r-", linewidth=0.5)
ax.plot(positions2[:, 0], positions2[:, 1], "b-", linewidth=0.5)

# Initialize plot elements
vehicle_image = plt.imread("vehicle.png")
image1 = OffsetImage(vehicle_image, zoom=0.05)
image2 = OffsetImage(vehicle_image, zoom=0.05)
vehicle1 = AnnotationBbox(image1, positions1[0], frameon=False)
vehicle2 = AnnotationBbox(image2, positions2[0], frameon=False)
ax.add_artist(vehicle1)
ax.add_artist(vehicle2)

# Animation update function
def update(frame):
    time_ms = frame * 50
    index1 = np.searchsorted(times1, time_ms)
    index2 = np.searchsorted(times2, time_ms)

    position1 = positions1[index1]
    position2 = positions2[index2]
    heading1 = headings1[index1]
    heading2 = headings2[index2]

    vehicle1.xybox = position1
    vehicle2.xybox = position2
    trans1 = Affine2D().rotate_deg_around(position1[0], position1[1], -np.rad2deg(heading1))
    trans2 = Affine2D().rotate_deg_around(position2[0], position2[1], -np.rad2deg(heading2))
    image1.set_transform(trans1 + ax.transData)
    image2.set_transform(trans2 + ax.transData)

    return vehicle1, vehicle2

# Create animation
num_frames = int(collision_time_ms / 50)
ani = FuncAnimation(fig, update, frames=num_frames, interval=100, blit=True)

# Save animation as MP4
ani.save("collision_animation.mp4", fps=30, extra_args=["-vcodec", "libx264"])

