import json
import matplotlib.pyplot as plt
import numpy as np

# Load data from JSON file
with open('unreachable_config.json', 'r') as file:
    data = json.load(file)

unreachable_configs = data['UnreachableConfigurations']

# Create two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Set titles for the subplots
ax1.set_title('Unreachable Configurations')
ax2.set_title('Found Nodes')

# Generate unique colors for each configuration
colors = plt.cm.get_cmap('hsv', len(unreachable_configs))

for i, config in enumerate(unreachable_configs):
    start = config['Start']
    target = config['Target']
    found_nodes = config['FoundNodes']

    # Plot start and endpoint on the first subplot
    ax1.plot([start[0], target[0]], [start[1], target[1]], linestyle='-', marker='o', color=colors(i), label=f"Config {i+1}")

    # Plot found nodes on the second subplot
    if config['FoundConfiguration']:
        node_x = [node[0] for node in found_nodes]
        node_y = [node[1] for node in found_nodes]
        ax2.scatter(node_x, node_y, c=colors(i), label=f"Config {i+1}")

# Set labels for the subplots
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')

# Add legends
ax1.legend(title='Configurations', loc='upper left')
ax2.legend(title='Configurations', loc='upper left')

# Show the plots
plt.savefig("../all_unreachable_configs.png")
plt.close()
