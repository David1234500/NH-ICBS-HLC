import os
import json
import numpy as np
import matplotlib.pyplot as plt

def process_results_json(results_json):
    data = {}
    infeasible_count = 0

    for experiment in results_json:
        if experiment["feasible"]:
            vehicle_count = len(experiment["interprimitives"])
            if vehicle_count not in data:
                data[vehicle_count] = {
                    "times": [],
                    "nodes": [],
                    "infeasible_count": 0
                }
            data[vehicle_count]["times"].append(experiment["compute_time"] / 1000)
            data[vehicle_count]["nodes"].append(experiment["max_node_id"])
        else:
            infeasible_count += 1

    return data, infeasible_count

def process_directory(path):
    with open(os.path.join(path, "results.json"), "r") as file:
        results_json = json.load(file)
    return process_results_json(results_json)

folders_data = {}
for root, dirs, files in os.walk("."):
    if "results.json" in files:
        folder_data, infeasible_count = process_directory(root)
        folders_data[root] = (folder_data, infeasible_count)

sorted_folders_data = dict(sorted(folders_data.items()))

fig, axs = plt.subplots(4, figsize=(15, 20))

for i, (folder_name, (data, infeasible_count)) in enumerate(sorted_folders_data.items()):
    vehicle_counts = sorted(data.keys())
    times = [data[vehicle_count]["times"] for vehicle_count in vehicle_counts]
    nodes = [data[vehicle_count]["nodes"] for vehicle_count in vehicle_counts]
    infeasible_counts = [infeasible_count] * len(vehicle_counts)

    axs[0].boxplot(times, positions=[i], labels=[folder_name], widths=0.6)
    axs[0].set_xlabel("Experiment Folder")
    axs[0].set_ylabel("Time for Feasible Solution (s)")

    axs[1].boxplot(nodes, positions=[i], labels=[folder_name], widths=0.6)
    axs[1].set_xlabel("Experiment Folder")
    axs[1].set_ylabel("Evaluated Nodes for Feasible Solution")

    axs[2].bar(folder_name, infeasible_count, width=0.6 ,color="blue")
    axs[2].set_xlabel("Experiment Folder")
    axs[2].set_ylabel("Infeasible Experiments Count")

    avg_times = [np.mean(t) for t in times]
    axs[3].bar(folder_name, np.mean(avg_times), width=0.6 ,color="blue")
    axs[3].set_xlabel("Experiment Folder")
    axs[3].set_ylabel("Average Runtime (s)")

plt.tight_layout()
plt.savefig("combined_boxplots_and_infeasible_counts_sorted.png")
