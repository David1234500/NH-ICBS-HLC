import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt

def process_results_json(results_json):
    data = {}

    for experiment in results_json:
        if experiment["feasible"]:
            vehicle_count = len(experiment["interprimitives"])
            if vehicle_count not in data:
                data[vehicle_count] = {
                    "times": [],
                }
            data[vehicle_count]["times"].append(experiment["compute_time"] / 1000)

    return data

def process_directory(path):
    with open(os.path.join(path, "results.json"), "r") as file:
        results_json = json.load(file)
    return process_results_json(results_json)

order_file = sys.argv[1] if len(sys.argv) > 1 else 'order.json'

with open(order_file, 'r') as file:
    order = json.load(file)

folders_data = {}
for item in order['experiments']:
    folder_name = item['folder_name']
    if os.path.exists(folder_name) and "results.json" in os.listdir(folder_name):
        folder_data = process_directory(folder_name)
        folders_data[item['experiment_name']] = folder_data

sorted_folders_data = dict(sorted(folders_data.items(), key=lambda x: next(i['order'] for i in order['experiments'] if i['experiment_name'] == x[0])))

fig, axs = plt.subplots(figsize=(12, 8))

experiment_names = []
avg_times_all = []
min_times_all = []
max_times_all = []

for i, (experiment_name, data) in enumerate(sorted_folders_data.items()):
    vehicle_counts = sorted(data.keys())
    times = [data[vehicle_count]["times"] for vehicle_count in vehicle_counts]

    # Store average, min, and max times
    avg_times = [np.mean(t) for t in times]
    min_times = [np.min(t) for t in times]
    max_times = [np.max(t) for t in times]

    experiment_names.append(experiment_name)
    avg_times_all.append(np.mean(avg_times))
    min_times_all.append(np.min(min_times))
    max_times_all.append(np.max(max_times))

# Plot average, min, and max times
axs.plot(experiment_names, avg_times_all, label="Average", marker='o')
axs.plot(experiment_names, min_times_all, label="Min", marker='o')
axs.plot(experiment_names, max_times_all, label="Max", marker='o')
axs.set_xlabel(order['x_axis_label'])
axs.set_ylabel("Time for Feasible Solution (s)")

axs.legend()

plt.tight_layout()
plt.savefig(order['output_file'])
