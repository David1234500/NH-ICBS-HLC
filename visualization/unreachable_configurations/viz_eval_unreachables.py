import json
import matplotlib.pyplot as plt
import numpy as np
import sys 

# Read unreachable configurations from JSON file
with open(sys.argv[1], "r") as file:
    unreachable_configs = json.load(file)

# Determine the grid size from the unreachable configurations
max_x = max(config["target_pose"]["x"] for config in unreachable_configs)
max_y = max(config["target_pose"]["y"] for config in unreachable_configs)
max_a = max(config["target_pose"]["a"] for config in unreachable_configs)

# Create the entire node grid
grid = np.zeros((max_x + 1, max_y + 1, max_a + 1))

# Highlight unreachable configurations in red
for config in unreachable_configs:
    x = config["target_pose"]["x"]
    y = config["target_pose"]["y"]
    a = config["target_pose"]["a"]
    grid[x, y, a] = 1

# Visualize the grid using matplotlib
fig, axes = plt.subplots(max_a + 1, figsize=(10, 10 * (max_a + 1)))

for a in range(max_a + 1):
    axes[a].imshow(grid[:, :, a].T, cmap='coolwarm', origin='lower')
    axes[a].set_title(f"Angle {a}")
    axes[a].set_xticks(np.arange(-0.5, max_x + 1, 1), minor=True)
    axes[a].set_yticks(np.arange(-0.5, max_y + 1, 1), minor=True)
    axes[a].grid(which='minor', color='black', linestyle='-', linewidth=1)

    # Add arrows for headings
    center_x = max_x // 2
    center_y = max_y // 2
    radians = (a / 12) * 2 * np.pi
    dx = 7 * np.cos(radians)
    dy = 7 * np.sin(radians)
    axes[a].arrow(center_x, center_y, dx, dy, head_width=2, head_length=2, fc='black', ec='black')

plt.tight_layout()
plt.savefig("../unreachables.png")
