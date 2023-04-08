import os
import json
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def plot_track(ax, track, times, title, colors):
    xs = [p["x"] for p in track]
    ys = [p["y"] for p in track]
    
    ax.scatter(xs, ys, c=colors, cmap=cm.jet)
    ax.set_title(title)

def visualize(json_file):
    with open(json_file, "r") as f:
        data = json.load(f)

    coarse_checks = data["coarse_checks"]
    precise_checks = data["precise_checks"]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    for check in coarse_checks:
        pose1 = check["pose1"]
        pose2 = check["pose2"]
        time = check["pose1"]["t"]
        plot_track(ax1, [pose1, pose2], [time, time], "Coarse Check", [time, time])

    for check in precise_checks:
        pose1 = check["pose1"]
        pose2 = check["pose2"]
        time = check["pose1"]["t"]
        plot_track(ax2, [pose1, pose2], [time, time], "Fine Check", [time, time])

    fig.tight_layout()
    plt.savefig("../"+os.path.splitext(json_file)[0] + ".png")
    plt.close()

if __name__ == "__main__":
    json_files = glob.glob("debug_collision_*.json")

    for json_file in json_files:
        visualize(json_file)
