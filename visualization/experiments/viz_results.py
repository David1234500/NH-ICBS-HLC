import json
import matplotlib.pyplot as plt
import numpy as np

def visualize_experiment_results(json_filename):
    with open(json_filename, "r") as f:
        results = json.load(f)

    n_experiments = len(results)

    # Extract data for bar graphs
    compute_times = [result["compute_time"] for result in results]
    max_node_ids = [result["max_node_id"] for result in results]
    feasibilities = [result["feasible"] for result in results]

    # Create bar graphs
    fig, (ax1, ax2) = plt.subplots(nrows=2, figsize=(10, 15), gridspec_kw={"height_ratios": [1, 3]})
    x = np.arange(n_experiments)
    width = 0.3
    colors = plt.cm.viridis(np.linspace(0, 1, n_experiments))

    ax1.bar(x - width/2, max_node_ids, width, label="Max Node ID", color=colors)
    ax1.set_xticks(x)
    ax1.set_xticklabels([f"Exp {i}" for i in range(n_experiments)])
    ax1.set_ylabel("Max Node ID")
    ax1.legend(loc="upper left")

    ax1_2 = ax1.twinx()
    ax1_2.bar(x + width/2, compute_times, width, label="Compute Time", color=["blue" if f else "red" for f in feasibilities])
    ax1_2.set_ylabel("Compute Time (ms)")
    ax1_2.legend(loc="upper right")

    # Plot trajectories
    plotted_labels = set()
    for i, result in enumerate(results):
        if result["feasible"]:
            for _, traj in result["interprimitives"].items():
                x_traj = [p["posx"] for p in traj]
                y_traj = [p["posy"] for p in traj]
                label = f"Exp {i}" if f"Exp {i}" not in plotted_labels else None
                ax2.plot(x_traj, y_traj, color=colors[i], label=label)
                if label:
                    plotted_labels.add(label)

    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.legend()

    # Save the figure
    plt.savefig("experiments_visualization.png")

if __name__ == "__main__":
    visualize_experiment_results("results.json")
