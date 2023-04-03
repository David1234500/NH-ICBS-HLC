import json
import matplotlib.pyplot as plt
import os
import glob

def visualize_unreachable_configuration(file_path):
    # Load data from JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    start = data['Start']
    target = data['Target']
    found_nodes = data['FoundNodes']

    # Create two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # Set titles for the subplots
    ax1.set_title(f"Start & Target\nStart: {start}\nTarget: {target}")
    ax2.set_title(f"Found Nodes\nStart: {start}\nTarget: {target}")

    # Plot start and target on the first subplot
    ax1.plot([start[0], target[0]], [start[1], target[1]], linestyle='-', marker='o', label='Start & Target')

    # Plot found nodes on the second subplot
    if data['FoundConfiguration']:
        node_x = [node[0] for node in found_nodes]
        node_y = [node[1] for node in found_nodes]
        ax2.scatter(node_x, node_y, label='Found Nodes')

    # Add the start and target nodes to the second subplot
    ax2.plot([start[0], target[0]], [start[1], target[1]], linestyle='-', marker='o', label='Start & Target')

    # Set labels for the subplots
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')

    # Add legends
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    # Save the plot as an image with the corresponding ID
    output_filename = os.path.join('..', os.path.basename(file_path).replace('.json', '.png'))
    plt.savefig(output_filename)
    plt.close(fig)

# Load all JSON files with the prefix "unreachable_configuration" and call the function for each file
for json_file in glob.glob("unreachable_configuration*.json"):
    visualize_unreachable_configuration(json_file)
