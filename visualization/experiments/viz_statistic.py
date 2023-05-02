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
                    "total_time": 0,
                    "total_nodes": 0,
                    "feasible_count": 0
                }
            data[vehicle_count]["total_time"] += experiment["compute_time"] / 1000
            data[vehicle_count]["total_nodes"] += experiment["max_node_id"]
            data[vehicle_count]["feasible_count"] += 1
        else:
            infeasible_count += 1

    if not data:
        return []

    processed_data = []
    for vehicle_count, info in data.items():
        avg_feasible_time = info["total_time"] / info["feasible_count"]
        avg_feasible_nodes = info["total_nodes"] / info["feasible_count"]

        processed_data.append({
            "vehicle_count": vehicle_count,
            "avg_feasible_time": avg_feasible_time,
            "infeasible_count": infeasible_count,
            "avg_feasible_nodes": avg_feasible_nodes
        })

    return processed_data

def process_directory(path):
    with open(os.path.join(path, "results.json"), "r") as file:
        results_json = json.load(file)
    return process_results_json(results_json)

data = []
for root, dirs, files in os.walk("."):
    if "results.json" in files:
        data.extend(process_directory(root))

data.sort(key=lambda x: x["vehicle_count"])

vehicle_counts = np.array([d["vehicle_count"] for d in data])
avg_feasible_times = np.array([d["avg_feasible_time"] for d in data])
infeasible_counts = np.array([d["infeasible_count"] for d in data])
evaluated_nodes = np.array([d["avg_feasible_nodes"] for d in data])

fig, axs = plt.subplots(3, figsize=(10, 10))

# Fit and plot polynomials
poly_coeff1 = np.polyfit(vehicle_counts, avg_feasible_times, 3)
poly_fit1 = np.poly1d(poly_coeff1)
axs[0].plot(vehicle_counts, avg_feasible_times, marker='o')
axs[0].plot(vehicle_counts, poly_fit1(vehicle_counts), label=f'Poly fit: {poly_coeff1}')
axs[0].legend()
axs[0].set_xlabel("Vehicle Count")
axs[0].set_ylabel("Average Time for Feasible Solution (s)")

poly_coeff2 = np.polyfit(vehicle_counts, evaluated_nodes, 3)
poly_fit2 = np.poly1d(poly_coeff2)
axs[1].plot(vehicle_counts, evaluated_nodes, marker='o')
axs[1].plot(vehicle_counts, poly_fit2(vehicle_counts), label=f'Poly fit: {poly_coeff2}')
axs[1].legend()
axs[1].set_xlabel("Vehicle Count")
axs[1].set_ylabel("Evaluated Nodes for Feasible Solution")

poly_coeff3 = np.polyfit(vehicle_counts, infeasible_counts, 3)
poly_fit3 = np.poly1d(poly_coeff3)
axs[2].plot(vehicle_counts, infeasible_counts, marker='o')
axs[2].plot(vehicle_counts, poly_fit3(vehicle_counts), label=f'Poly fit: {poly_coeff3}')
axs[2].legend()
axs[2].set_xlabel("Vehicle Count")
axs[2].set_ylabel("Infeasible Experiments Count")

plt.tight_layout()
plt.savefig("../experiment_stats.png")

