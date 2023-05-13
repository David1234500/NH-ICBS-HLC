import os
import json
import numpy as np
import matplotlib.pyplot as plt

def process_results_json(results_json):
    data = {}

    for experiment in results_json:
        if "interprimitives" in experiment:
            vehicle_count = len(experiment["interprimitives"])
            if vehicle_count not in data:
                data[vehicle_count] = {
                    "times": [],
                }

            if experiment["feasible"]:
                data[vehicle_count]["times"].append(experiment["compute_time"] / 1000)

    return data

def process_directory(path):
    with open(os.path.join(path, "results.json"), "r") as file:
        results_json = json.load(file)
    return process_results_json(results_json)

data = {}
for root, dirs, files in os.walk("."):
    if "results.json" in files:
        dir_data = process_directory(root)
        for vehicle_count, info in dir_data.items():
            if vehicle_count not in data:
                data[vehicle_count] = {
                    "times": [],
                }
            data[vehicle_count]["times"].extend(info["times"])

vehicle_counts = sorted(data.keys())
times_data = [data[vc]["times"] for vc in vehicle_counts]

fig, axs = plt.subplots(2, figsize=(12, 8))

# Create average line plot
avg_times = [np.mean(times) for times in times_data]
axs[0].plot(vehicle_counts, avg_times, marker='o')
axs[0].set_xlabel("Vehicle Count")
axs[0].set_ylabel("Average Time for Feasible Solution (s)")

# Create min, max, average line plot
min_times = [np.min(times) for times in times_data]
max_times = [np.max(times) for times in times_data]
axs[1].plot(vehicle_counts, avg_times, label="Average", marker='o')
axs[1].plot(vehicle_counts, min_times, label="Min", marker='o')
axs[1].plot(vehicle_counts, max_times, label="Max", marker='o')
axs[1].set_xlabel("Vehicle Count")
axs[1].set_ylabel("Time for Feasible Solution (s)")
axs[1].legend()

plt.tight_layout()
plt.savefig("../experiment_stats_lines.png")
