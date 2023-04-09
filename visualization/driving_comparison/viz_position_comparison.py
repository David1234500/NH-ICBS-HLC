import json
import matplotlib.pyplot as plt
import math
import sys
import numpy as np
from scipy.interpolate import interp1d

f_actual = open(sys.argv[1], "r")
x_actual = f_actual.read()
pose_data = json.loads(x_actual)

time_ref_ms = pose_data["time_ref_ms"]

actual_data = []
for node in pose_data["actual"][0]["path"]:
    if node["time_ms"] < 1000000:
        actual_data.append((node["px"], node["py"], node["ph"], node["pv"], node["time_ms"]))

ref_data = []
for node in pose_data["ref"][0]["path"]:
    ref_data.append((node["px"], node["py"], node["ph"], node["pv"], node["time_ms"]))

fig = plt.figure(constrained_layout=True, figsize=(20,15))
fig.suptitle('Comparison between Reference and Measured Values')

subfigs = fig.subplots(nrows=5, ncols=1)
ref_data_np = np.array(ref_data)
actual_data_np = np.array(actual_data)

indices = [3, 2, 1, 0]
titles = ["Velocity", "Heading", "X Position", "Y Position"]
y_labels = ["Velocity (cm/s)", "Heading (Radians)", "X Position (cm)", "Y Position (cm)"]

for i, idx in enumerate(indices):
    ref_values = ref_data_np[:, idx]
    actual_values = actual_data_np[:, idx]
    time_ref = ref_data_np[:, 4]
    time_actual = actual_data_np[:, 4]

    if idx == 2:  # Adjust heading by -2 * pi
        ref_values = ref_values - 2 * math.pi

    # Interpolate actual values to match reference time values
    interp_func = interp1d(time_actual, actual_values, fill_value="extrapolate")
    actual_values_interp = interp_func(time_ref)

    # Calculate deviation and cumulative deviation
    deviation = np.abs(ref_values - actual_values_interp)
    cumulative_deviation = np.cumsum(deviation) / 100

    # Plot reference and actual values
    subfigs[i].plot(time_ref, ref_values, label="Reference")
    subfigs[i].plot(time_actual, actual_values, label="Actual")

    # Plot cumulative deviation
    subfigs[i].plot(time_ref, cumulative_deviation, label="Cumulative Deviation", linestyle='--')

    # Set title, labels, and legend
    subfigs[i].set_title(f"{titles[i]} over Time Comparison (Total Deviation: {cumulative_deviation[-1]:.2f})")
    subfigs[i].set_xlabel("Time (ms)")
    subfigs[i].set_ylabel(y_labels[i])
    subfigs[i].legend()

# Additional subplot for position difference in both X and Y axes
ref_values_x = ref_data_np[:, 1]
actual_values_x = actual_data_np[:, 1]
ref_values_y = ref_data_np[:, 0]
actual_values_y = actual_data_np[:, 0]

# Interpolate actual values to match reference time values
interp_func_x = interp1d(time_actual, actual_values_x, fill_value="extrapolate")
actual_values_x_interp = interp_func_x(time_ref)
interp_func_y = interp1d(time_actual, actual_values_y, fill_value="extrapolate")
actual_values_y_interp = interp_func_y(time_ref)

# Calculate deviation and cumulative deviation
deviation_x = np.abs(ref_values_x - actual_values_x_interp)
deviation_y = np.abs(ref_values_y - actual_values_y_interp)
cumulative_deviation_x = np.cumsum(deviation_x)
cumulative_deviation_y = np.cumsum(deviation_y)

# Plot cumulative deviation for X and Y positions
subfigs[4].plot(time_ref, cumulative_deviation_x, label="X Position Cumulative Deviation", linestyle='--')
subfigs[4].plot(time_ref, cumulative_deviation_y, label="Y Position Cumulative Deviation", linestyle=':')

# Set title, labels, and legend
subfigs[4].set_title(f"Position Deviation over Time (Total Deviation X: {cumulative_deviation_x[-1]:.2f}, Y: {cumulative_deviation_y[-1]:.2f})")
subfigs[4].set_xlabel("Time (ms)")
subfigs[4].set_ylabel("Position Deviation (cm)")
subfigs[4].legend()


plt.savefig("../comparison_driving.png")
