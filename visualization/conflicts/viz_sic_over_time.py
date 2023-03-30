import sys
import json
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import os


def load_json_files(start_index, end_index):
    data = []
    for i in range(start_index, end_index + 1):
        filename = f"node{i}.json"
        if not os.path.exists(filename):
            continue

        with open(filename, "r") as f:
            data.append(json.load(f))
    return data


def plot_data(data):
    fig, axs = plt.subplots(len(data) + 1, 1, figsize=(10, 5 * (len(data) + 1)), sharex=True)
    fig.subplots_adjust(hspace=0.5)

    sic_values = [d["sic"] for d in data]
    node_indices = [d["node_id"] for d in data]

    cmap = cm.get_cmap("viridis")
    norm = plt.Normalize(min(sic_values), max(sic_values))

    # Upper plot
    axs[0].bar(node_indices, sic_values, color=cmap(norm(sic_values)))
    axs[0].set_title("SIC Values")
    axs[0].set_ylabel("SIC")
    axs[0].set_xlabel("Node Index")

    # Lower subplots
    for i, d in enumerate(data):
        ax = axs[i + 1]

        for vpath in d["multipath"]:
            interprimitive = vpath["interprimitive"]
            x = [p["x"] for p in interprimitive]
            y = [p["y"] for p in interprimitive]

            ax.plot(x, y, label=f"Vehicle {vpath['id']}")
            ax.set_title(f"Node {d['node_id']}")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.legend()

        ax.set_facecolor(cmap(norm(d["sic"])))

    plt.savefig("../sic_over_time.png")


def main(start_index, end_index):
    data = load_json_files(start_index, end_index)
    plot_data(data)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_data.py <start_index> <end_index>")
        sys.exit(1)

    start_index = int(sys.argv[1])
    end_index = int(sys.argv[2])

    main(start_index, end_index)
