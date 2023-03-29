import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import sys

# Load JSON data
with open(sys.argv[1], 'r') as f:
    data = json.load(f)

# Initialize data lists
time_indices = []
time_values = []
interprimitive_indices = []
interprimitive_positions = []
bnodes_positions = []
colors = []

# Parse JSON data and populate data lists
for vehicle in data["multipath"]:
    for i, interprimitive in enumerate(vehicle["interprimitive"]):
        time_indices.append(interprimitive["ti"])
        time_values.append(interprimitive["t"])
        interprimitive_indices.append(i)
        interprimitive_positions.append([interprimitive["x"], interprimitive["y"]])
        bnodes_positions.append([interprimitive["bnode"]["x"], interprimitive["bnode"]["y"]])
        colors.append(interprimitive["t"])

# Normalize colors for colormap
t_max = max(colors)
norm_colors = [c / t_max for c in colors]

# Create custom colormap (blue to green)
cmap = LinearSegmentedColormap.from_list('blue_to_green', ['blue', 'green'])

# Create a figure
fig, axs = plt.subplots(3, 1, figsize=(8, 18))

# Create the first subplot (Time Index vs Interprimitive Index)
sc1 = axs[0].scatter(time_indices, interprimitive_indices, c=norm_colors, cmap=cmap)
axs[0].set_xlabel('Time Index')
axs[0].set_ylabel('Interprimitive Index')
axs[0].set_title('Time Index vs Interprimitive Index')
fig.colorbar(sc1, ax=axs[0], label='Time')

# Create the second subplot (Time vs Interprimitive Index)
sc2 = axs[1].scatter(time_values, interprimitive_indices, c=norm_colors, cmap=cmap)
axs[1].set_xlabel('Time (t)')
axs[1].set_ylabel('Interprimitive Index')
axs[1].set_title('Time (t) vs Interprimitive Index')
fig.colorbar(sc2, ax=axs[1], label='Time')

# Create the third subplot (Interprimitive Positions and Base Nodes)
sc3 = axs[2].scatter(*zip(*interprimitive_positions), c=norm_colors, cmap=cmap, label='Interprimitive Positions')
axs[2].scatter(*zip(*bnodes_positions), c='red', marker='x', label='Base Nodes')
axs[2].set_xlabel('X Position')
axs[2].set_ylabel('Y Position')
axs[2].set_title('Interprimitive Positions and Base Nodes')
fig.colorbar(sc3, ax=axs[2], label='Time')
axs[2].legend()

# Display the plots
plt.tight_layout()
plt.show()
