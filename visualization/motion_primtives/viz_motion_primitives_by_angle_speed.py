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
subfigs = fig.subfigures(nrows=y["info"]["size_s"], ncols=1)
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

    grid[edge["source"]["s"]][edge["source"]["a"]].set_aspect('equal')
    grid[edge["source"]["s"]][edge["source"]["a"]].plot(ex,ey)
    grid[edge["source"]["s"]][edge["source"]["a"]].set_xlim([-100,100])
    grid[edge["source"]["s"]][edge["source"]["a"]].set_xlabel("cm")
    grid[edge["source"]["s"]][edge["source"]["a"]].set_ylabel("cm")
    grid[edge["source"]["s"]][edge["source"]["a"]].set_ylim([-100,100])


plt.show()

