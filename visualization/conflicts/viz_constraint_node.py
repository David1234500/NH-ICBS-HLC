import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import sys
from matplotlib.colors import Normalize, ListedColormap, LinearSegmentedColormap
import glob
import os

def visualize_constraints(file_name, lower_threshold, time_interval=500):
    # Load the JSON data from file
    with open(file_name, "r") as f:
        data = json.load(f)

    # Extract interprimitive positions
    interprimitives = [vehicle["interprimitive"] for vehicle in data["multipath"]]
    vehicle_ids = [vehicle["id"] for vehicle in data["multipath"]]

    # Plot vehicle trajectories
    plt.figure(figsize=(15, 10))

    plt.subplot(2, 1, 1)
    
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
            plt.plot(x_values, y_values, color=colors[i])

        # Add periodic time labels
        for point in interprimitive:
            if point["t"] % time_interval == 0:
                plt.text(point["x"], point["y"], f"t: {point['t']}")

    # Plot conflicts
    if "avoid" in data:
        for conflict in data["avoid"]:
            plt.scatter(conflict["x"], conflict["y"], s=50, marker="x", c="red")
            plt.text(conflict["x"], conflict["y"], f"id: {conflict['id']}, t: {conflict['t']}")

    plt.title("Vehicle Trajectories and Conflicts")
    plt.xlabel("X")
    plt.ylabel("Y")


    # Plot inter-vehicle distance over time
    plt.subplot(2, 1, 2)

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

            plt.plot(common_times, distances, label=f"Vehicle {vehicle_ids[i]}-{vehicle_ids[j]}")

            # Highlight when the distance crosses the lower threshold
            below_threshold = [t for t, d in zip(common_times, distances) if d <= lower_threshold]
            for t in below_threshold:
                plt.axvline(x=t, linestyle="--", color="red", alpha=0.5)

    plt.title("Inter-vehicle Distance Over Time")
    plt.xlabel("Time (ms)")
    plt.ylabel("Distance")
    plt.legend()

    plt.tight_layout()
    plt.savefig("../"+ str(data["node_id"]) + ".png")
    plt.close()

if __name__ == "__main__":
    prefix = 'node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        visualize_constraints(filename, lower_threshold=20)