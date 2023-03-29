import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import sys

# Load JSON data
with open(sys.argv[1], 'r') as f:
    data = json.load(f)

# Initialize data lists
path_positions = []
interprimitive_positions = []
bnodes_positions = []
colors = []

# Parse JSON data and populate data lists
for pnode in data["path"]:
    path_positions.append([pnode["x"], pnode["y"]])

for i, interprimitive in enumerate(data["interprimitive"]):
    interprimitive_positions.append([interprimitive["x"], interprimitive["y"]])
    bnodes_positions.append([interprimitive["bnode"]["x"], interprimitive["bnode"]["y"]])
    colors.append(interprimitive["ti"])

# Create custom colormap
cmap = plt.get_cmap('viridis', max(colors) + 1)

# Create a figure
fig, axs = plt.subplots(3, 1, figsize=(8, 18))

# Create the first subplot (Path Positions)
axs[0].scatter(*zip(*path_positions))
axs[0].plot(*zip(*path_positions), linestyle='-', linewidth=1, color='gray')
axs[0].set_xlabel('X Position')
axs[0].set_ylabel('Y Position')
axs[0].set_title('Path Positions')

# Create the second subplot (Interprimitive Positions and Base Nodes)
sc = axs[1].scatter(*zip(*interprimitive_positions), c=colors, cmap=cmap, label='Interprimitive Positions')
axs[1].scatter(*zip(*bnodes_positions), c='red', marker='x', label='Base Nodes')
axs[1].set_xlabel('X Position')
axs[1].set_ylabel('Y Position')
axs[1].set_title('Interprimitive Positions and Base Nodes')
fig.colorbar(sc, ax=axs[1], label='Path Depth Index')
axs[1].legend()

# Create the third subplot (Path Positions Colored by Index)
path_colors = [cmap(i) for i in range(len(path_positions), 0, -1)]
axs[2].scatter(*zip(*path_positions), c=path_colors)
axs[2].plot(*zip(*path_positions), linestyle='-', linewidth=1, color='gray')
axs[2].set_xlabel('X Position')
axs[2].set_ylabel('Y Position')
axs[2].set_title('Path Positions Colored by Index')

# Display the plots
plt.tight_layout()
plt.show()
