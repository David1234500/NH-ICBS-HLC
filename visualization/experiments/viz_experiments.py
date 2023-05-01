import json
import matplotlib.pyplot as plt
import math

def visualize_experiments(experiments_filename, output_filename):
    with open(experiments_filename) as experiments_file:
        experiments = json.load(experiments_file)

    fig, axs = plt.subplots(len(experiments), figsize=(8, 4 * len(experiments)))

    for idx, experiment in enumerate(experiments):
        starts = experiment["starts"]
        targets = experiment["targets"]

        start_xs = [start["x"] for start in starts]
        start_ys = [start["y"] for start in starts]

        target_xs = [target["x"] for target in targets]
        target_ys = [target["y"] for target in targets]

        if len(experiments) == 1:
            ax = axs
        else:
            ax = axs[idx]

        ax.scatter(start_xs, start_ys, c="blue", marker="o", label="Start")
        ax.scatter(target_xs, target_ys, c="red", marker="^", label="Target")

        # Draw lines between associated start and target positions
        for start_x, start_y, target_x, target_y, start, target in zip(start_xs, start_ys, target_xs, target_ys, starts, targets):
            ax.plot([start_x, target_x], [start_y, target_y], c="green", linestyle="--")

            start_heading_rad = (start["a"] / 12) * 2 * math.pi
            target_heading_rad = (target["a"] / 12) * 2 * math.pi

            ax.arrow(start_x, start_y, math.cos(start_heading_rad), math.sin(start_heading_rad), width=0.5, color="blue")
            ax.arrow(target_x, target_y, math.cos(target_heading_rad), math.sin(target_heading_rad), width=0.5, color="red")

        ax.set_title(f"Experiment {idx + 1}")
        ax.legend()

    plt.tight_layout()
    plt.savefig(output_filename)

experiments_filename = "experiments.json"
output_filename = "experiments.png"

visualize_experiments(experiments_filename, output_filename)
