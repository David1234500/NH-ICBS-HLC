import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.axes_grid1 import make_axes_locatable
import sys
from matplotlib.colors import Normalize
import glob
import os

def moving_average(a, n=3):
    window = np.ones(n) / n
    return np.convolve(a, window, mode='valid')

def visualize_constraints(file_name, lower_threshold, time_interval=500):
    # Load the JSON data from file
    with open(file_name, "r") as f:
        data = json.load(f)

    # Extract interprimitive positions
    interprimitives = [vehicle["interprimitive"] for vehicle in data["multipath"]]
    vehicle_ids = [vehicle["id"] for vehicle in data["multipath"]]

    # Plot vehicle trajectories
    plt.figure(figsize=(20, 12))

    ax1 = plt.subplot(2, 1, 1)
    
    # Find the maximum time across all vehicles
    max_time = max(point["t"] for interprimitive in interprimitives for point in interprimitive)

    # Create a colormap for the vehicle paths
    cmap = cm.get_cmap("viridis")
    norm = Normalize(vmin=0, vmax=max_time)

    for interprimitive in interprimitives:
        # Generate a list of colors corresponding to the time values of the interprimitive points
        colors = [cmap(norm(point["t"])) for point in interprimitive]
        
        # Plot vehicle trajectories with gradually changing colors
        for i in range(len(interprimitive) - 1):
            x_values = [interprimitive[i]["x"], interprimitive[i + 1]["x"]]
            y_values = [interprimitive[i]["y"], interprimitive[i + 1]["y"]]
            ax1.plot(x_values, y_values, color=colors[i], linewidth=3)

        # Add periodic time labels
        for point in interprimitive:
            if point["t"] % time_interval == 0:
                pass
                # ax1.text(point["x"], point["y"], f"t: {point['t']}")

    # Plot conflicts
    if "avoid" in data:
        for conflict in data["avoid"]:
            ax1.scatter(conflict["x"], conflict["y"], s=200, marker="o", c="orange")
            ax1.text(conflict["x"], conflict["y"], f"id: {conflict['id']}, t: {conflict['t']}")

    ax1.set_title("Vehicle Trajectories and introduced Conflicts")
    ax1.set_xlabel("X [cm]")
    ax1.set_ylabel("Y [cm]")

    # Add colorbar legend for time
    divider = make_axes_locatable(ax1)
    cax = divider.append_axes("right", size="3%", pad=0.05)
    cbar = plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax)
    cbar.set_label('Time [ms]')

    # Plot inter-vehicle distance over time
    ax2 = plt.subplot(2, 1, 2)

    for i in range(len(vehicle_ids)):
        for j in range(i+1, len(vehicle_ids)):
            times_i = [point["t"] for point in interprimitives[i]]
            times_j = [point["t"] for point in interprimitives[j]]
            common_times = sorted(list(set(times_i) & set(times_j)))

            distances = []
            for t in common_times:
                point_i = next(point for point in interprimitives[i] if point["t"] == t)
                point_j = next(point for point in interprimitives[j] if point["t"] == t)

                distance = np.sqrt((point_i["x"] - point_j["x"])**2 + (point_i["y"] - point_j["y"])**2)
                distances.append(distance)

            # Apply moving average smoothing
            smoothed_distances = moving_average(distances)
            smoothed_times = common_times[: -2]

            ax2.plot(smoothed_times, smoothed_distances, label=f"Vehicle {vehicle_ids[i]}-{vehicle_ids[j]}", linewidth=2)

            # Highlight when the distance crosses the lower threshold
            below_threshold = [t for t, d in zip(smoothed_times, smoothed_distances) if d <= lower_threshold]
            for t in below_threshold:
                ax2.axvline(x=t, linestyle="--", color="red", alpha=0.5)

    ax2.set_title("Inter-vehicle Distance Over Time")
    ax2.set_xlabel("Time [ms]")
    ax2.set_ylabel("Distance [cm]")
    ax2.legend()

    divider2 = make_axes_locatable(ax2)
    cax2 = divider2.append_axes("bottom", size="3%", pad=0.7)
    cbar2 = plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax2, orientation='horizontal')
    cbar2.set_label('Time [ms]')

    plt.tight_layout()
    plt.savefig(str(data["node_id"]) + ".png")
    plt.close()

if __name__ == "__main__":
    prefix = 'node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        visualize_constraints(filename, lower_threshold=16)
