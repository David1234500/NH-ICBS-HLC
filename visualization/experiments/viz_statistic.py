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
                    "nodes": [],
                    "infeasible_count": 0
                }

            if experiment["feasible"]:
                data[vehicle_count]["times"].append(experiment["compute_time"] / 1000)
                data[vehicle_count]["nodes"].append(experiment["max_node_id"])
            else:
                data[vehicle_count]["infeasible_count"] += 1

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
                    "nodes": [],
                    "infeasible_count": 0
                }
            data[vehicle_count]["times"].extend(info["times"])
            data[vehicle_count]["nodes"].extend(info["nodes"])
            data[vehicle_count]["infeasible_count"] += info["infeasible_count"]

vehicle_counts = sorted(data.keys())
times_data = [data[vc]["times"] for vc in vehicle_counts]
nodes_data = [data[vc]["nodes"] for vc in vehicle_counts]
infeasible_counts = [data[vc]["infeasible_count"] for vc in vehicle_counts]

fig, axs = plt.subplots(3, figsize=(10, 10))

# Create boxplots
axs[0].boxplot(times_data, positions=vehicle_counts)
axs[0].set_xlabel("Vehicle Count")
axs[0].set_ylabel("Time for Feasible Solution (s)")

axs[1].boxplot(nodes_data, positions=vehicle_counts)
axs[1].set_xlabel("Vehicle Count")
axs[1].set_ylabel("Evaluated Nodes for Feasible Solution")

# Create bar plot for infeasible experiments count
axs[2].bar(vehicle_counts, infeasible_counts)
axs[2].set_xlabel("Vehicle Count")
axs[2].set_ylabel("Infeasible Experiments Count")

plt.tight_layout()
plt.savefig("../experiment_stats_boxplots.png")
