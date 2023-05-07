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

def visualize_constraints(file_name, lower_threshold, time_interval=0.5):
    # Load the JSON data from file
    with open(file_name, "r") as f:
        data = json.load(f)

    # Extract interprimitive positions
    interprimitives = [vehicle["interprimitive"] for vehicle in data["multipath"]]
    vehicle_ids = [vehicle["id"] for vehicle in data["multipath"]]

    # Plot vehicle trajectories
    plt.figure(figsize=(12, 8))

    ax1 = plt.subplot(1, 1, 1)
    ax1.set_xlim([0, 250])
    ax1.set_ylim([0, 250])

    ax1.set_xticks(range(0, 251, 25))
    ax1.set_yticks(range(0, 251, 25))

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
            if point["t"] % (time_interval * 1000) == 0:
                pass

    # Plot conflicts
    if "avoid" in data:
        for conflict in data["avoid"]:
            ax1.scatter(conflict["x"], conflict["y"], s=200, marker="o", c="red")

    ax1.set_title("Vehicle Trajectories and introduced Conflicts")
    ax1.set_xlabel("X [cm]")
    ax1.set_ylabel("Y [cm]")

    # Add colorbar legend for time
    divider = make_axes_locatable(ax1)
    cax = divider.append_axes("right", size="2%", pad=0.05)
    cbar = plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax)
    cbar.set_label('Time [ms]')

    plt.tight_layout()
    plt.savefig(str(data["node_id"]) + ".png")
    plt.close()

if __name__ == "__main__":
    prefix = 'node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        visualize_constraints(filename, lower_threshold=16)
