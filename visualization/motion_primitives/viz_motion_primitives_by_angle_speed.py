import json
import matplotlib.pyplot as plt
import math
import numpy as np
import sys

f = open("mp_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  

fig = plt.figure(constrained_layout=True)
fig.suptitle('Motion Primitives Visualisation')
grid = {}


# create 3x1 subfigs
subfigs = fig.subfigures(nrows=y["info"]["size_s"] * y["info"]["size_s"], ncols=1)
for row, subfig in enumerate(subfigs):
    subfig.suptitle(f'Source Velocity Level: {row}')
    grid[row] = {}

    # create 1x3 subplots per subfig
    axs = subfig.subplots(nrows=1, ncols=y["info"]["size_a"])
    for col, ax in enumerate(axs):
        ax.plot()
        ax.set_title(f'S Heading: {col}')
        grid[row][col] = ax
        

for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])

    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].set_aspect('equal')
    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].plot(ex,ey)
    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].set_xlim([-100,100])
    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].set_xlabel("cm")
    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].set_ylabel("cm")
    grid[2 * edge["source"]["s"] + edge["targeti"]["s"]][edge["source"]["a"]].set_ylim([-100,100])


plt.savefig("../motion_primitives_by_angle.png", dpi=500)

