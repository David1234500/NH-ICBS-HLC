import json
import matplotlib.pyplot as plt
import math
import os

def visualize_experiments(experiments_filename, output_folder):
    with open(experiments_filename) as experiments_file:
        experiments = json.load(experiments_file)

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for idx, experiment in enumerate(experiments):
        fig, ax = plt.subplots(figsize=(12, 8))

        starts = experiment["starts"]
        targets = experiment["targets"]

        start_xs = [start["x"] * 5 for start in starts]
        start_ys = [start["y"] * 5 for start in starts]

        target_xs = [target["x"] * 5 for target in targets]
        target_ys = [target["y"] * 5 for target in targets]

        ax.scatter(start_xs, start_ys, c="blue", marker="o", label="Start")
        ax.scatter(target_xs, target_ys, c="red", marker="^", label="Target")

        # Draw lines between associated start and target positions
        for start_x, start_y, target_x, target_y, start, target in zip(start_xs, start_ys, target_xs, target_ys, starts, targets):
            ax.plot([start_x, target_x], [start_y, target_y], c="green", linestyle="--",linewidth=2 )

            start_heading_rad = ((start["a"] / 12) * 2 * math.pi) 
            target_heading_rad = ((target["a"] / 12) * 2 * math.pi)

            ax.arrow(start_x, start_y, 2*math.cos(start_heading_rad), 2*math.sin(start_heading_rad), width=2, color="blue", )
            ax.arrow(target_x, target_y, 2*math.cos(target_heading_rad), 2*math.sin(target_heading_rad), width=2, color="red")

        ax.set_title(f"Experiment {idx + 1}")
        ax.legend()
        
        ax.set_xlabel('X Position [cm]')
        ax.set_ylabel('Y Position [cm]')

        ax.set_x

        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, f"experiment_{idx + 1}.png"))
        plt.close(fig)

experiments_filename = "experiments.json"
output_folder = "experiment_images"

visualize_experiments(experiments_filename, output_folder)
