import json
import matplotlib.pyplot as plt

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

        ax.set_title(f"Experiment {idx + 1}")
        ax.legend()

    plt.tight_layout()
    plt.savefig(output_filename)

experiments_filename = "experiments.json"
output_filename = "experiments.png"

visualize_experiments(experiments_filename, output_filename)
