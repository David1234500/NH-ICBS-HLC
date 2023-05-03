import json
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 3:
    print("Usage: python motion_primitives_visualization.py [source_velocity] [target_velocity]")
    sys.exit(1)

source_velocity = int(sys.argv[1])
target_velocity = int(sys.argv[2])

with open("mp_state_graph.json", "r") as f:
    y = json.load(f)

fig = plt.figure(figsize=(20,20),constrained_layout=True)
fig.suptitle('Motion Primitives Visualization')
grid = {}

# create subfigures
subfigs = fig.subfigures(nrows=y["info"]["size_a"], ncols=1)
for row, subfig in enumerate(subfigs):
    subfig.suptitle(f'Source Heading: {row} degrees')
    grid[row] = {}

    # create subplots per subfig
    axs = subfig.subplots(nrows=1, ncols=y["info"]["size_a"])
    for col, ax in enumerate(axs):
        ax.plot()
        ax.set_title(f'Target Heading: {col} degrees')
        grid[row][col] = ax
        ax.set_aspect('equal')
        ax.set_xlim([-100, 100])
        ax.set_ylim([-100, 100])

for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])

    if edge["targeti"]["s"] != target_velocity or edge["source"]["s"] != source_velocity:
        continue

    grid[edge["source"]["a"]][edge["targeti"]["a"]].plot(ex, ey)
    grid[edge["source"]["a"]][edge["targeti"]["a"]].set_xlabel("X position (cm)")
    grid[edge["source"]["a"]][edge["targeti"]["a"]].set_ylabel("Y position (cm)")

plt.savefig(f"../mps_from_{source_velocity}_to_{target_velocity}.png")
