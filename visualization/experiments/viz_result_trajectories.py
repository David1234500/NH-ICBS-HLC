import json
import matplotlib.pyplot as plt
import math

def visualize_experiment_results(json_filename):
    with open(json_filename, "r") as f:
        results = json.load(f)

    for i, result in enumerate(results):
        fig, ax = plt.subplots(figsize=(12, 8))
        start_label_added = False
        target_label_added = False

        if result["feasible"]:
            for _, traj in result["interprimitives"].items():
                x_traj = [p["posx"] for p in traj]
                y_traj = [p["posy"] for p in traj]
                ax.plot(x_traj, y_traj)

                start = {"x": x_traj[0], "y": y_traj[0], "a": traj[0]["h"]}
                target = {"x": x_traj[-1], "y": y_traj[-1], "a": traj[-1]["h"]}

                if not start_label_added:
                    ax.scatter(start["x"], start["y"], c="blue", marker="o", label="Start")
                    start_label_added = True
                else:
                    ax.scatter(start["x"], start["y"], c="blue", marker="o")

                if not target_label_added:
                    ax.scatter(target["x"], target["y"], c="red", marker="^", label="Target")
                    target_label_added = True
                else:
                    ax.scatter(target["x"], target["y"], c="red", marker="^")

                start_heading_rad = start["a"] 
                target_heading_rad = target["a"]

                ax.arrow(start["x"], start["y"], 2 * math.cos(start_heading_rad), 2 * math.sin(start_heading_rad), width=2, color="blue")
                ax.arrow(target["x"], target["y"], 2 * math.cos(target_heading_rad),2 * math.sin(target_heading_rad), width=2, color="red")

        ax.set_xlabel("X Position [cm]")
        ax.set_ylabel("Y Position [cm]")
        ax.set_title(f"Experiment {i + 1}")
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(f"experiment_{i + 1}_trajectory.png")

if __name__ == "__main__":
    visualize_experiment_results("results.json")
