import json
import matplotlib.pyplot as plt
import math
import numpy as np

f = open("proxy_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  

fig = plt.figure(constrained_layout=True)
fig.suptitle('Motion Primitives Visualisation')
grid = {}

# create 3x1 subfigs
subfigs = fig.subfigures(nrows=y["info"]["size_a"], ncols=1)
for row, subfig in enumerate(subfigs):
    # subfig.suptitle(f'Source Heading: {row}')
    grid[row] = {}

    # create 1x3 subplots per subfig
    axs = subfig.subplots(nrows=1, ncols=y["info"]["size_a"])
    for col, ax in enumerate(axs):
        ax.plot()
        # ax.set_title(f'Target Heading: {col}')
        grid[row][col] = ax
        ax.set_aspect('equal')
        ax.set_xlim([-100,100])
        ax.set_ylim([-100,100])



for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])

    if edge["targeti"]["s"] != 1 or edge["source"]["s"] != 1:
        continue
    
    grid[edge["source"]["a"]][edge["targeti"]["a"]].plot(ex,ey)
    # grid[edge["source"]["a"]][edge["targeti"]["a"]].set_xlabel("cm")
    # grid[edge["source"]["a"]][edge["targeti"]["a"]].set_ylabel("cm")
    

plt.show()

