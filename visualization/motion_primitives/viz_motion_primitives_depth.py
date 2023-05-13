import json
import matplotlib.pyplot as plt
import argparse
from collections import deque
import numpy as np

def plot_primitives(json_file, source_a, source_s, depth):
    # Load the JSON file
    with open(json_file) as f:
        data = json.load(f)

    # Use a queue to keep track of the next sources to process
    sources_queue = deque([(source_a, source_s, 0, 0, 0)])  # Each entry is a tuple (source_a, source_s, current_depth, x_offset, y_offset)

    plt.figure(figsize=(12,8))

    # Get the extent of the map
    map_coords = [point['pose'] for point in data['map']]
    x_values = [coord['x'] for coord in map_coords]
    y_values = [coord['y'] for coord in map_coords]
    x_min, x_max = 0, 150
    y_min, y_max = -130,130

    # Set up the grid
    plt.xticks(np.arange(x_min, x_max, 10))  # 5 cm spacing
    plt.yticks(np.arange(y_min, y_max, 10))  # 5 cm spacing
    plt.grid(True, which='both', color='grey', linestyle='-', linewidth=0.5)

    # Use a queue to keep track of the next sources to process
    sources_queue = deque([(source_a, source_s, 0, 0, 0)])  # Each entry is a tuple (source_a, source_s, current_depth, x_offset, y_offset)

    while sources_queue:
        source_a, source_s, current_depth, x_offset, y_offset = sources_queue.popleft()
        
        if current_depth > depth:
            break 

        # Get the edges with the right source a and s
        relevant_edges = [edge for edge in data['edges'] if edge['source']['a'] == source_a and edge['source']['s'] == source_s and edge['targeti']['s'] == 2]
        print(len(sources_queue))

        # For each edge, plot the curve
        for edge in relevant_edges:
            x_values = [point['x'] + x_offset for point in edge['curve']]
            y_values = [point['y'] + y_offset for point in edge['curve']]
            plt.plot(x_values, y_values, linewidth=1.0)

            # Compute the offsets for the next MP
            next_x_offset = x_values[-1]
            next_y_offset = y_values[-1]

            # Add the target to the queue for the next depth level
            if current_depth < depth:
                sources_queue.append((edge['targeti']['a'], 2, current_depth + 1, next_x_offset, next_y_offset))

    plt.title(f"Motion Primitives Reachability Graph with Depth {depth + 1}")
    plt.xlabel("X Position[cm]")
    plt.ylabel("Y Position[cm]")
    plt.legend()
    plt.savefig("../mp_reach_graph.png")

def main():
    parser = argparse.ArgumentParser(description='Plot motion primitives from a JSON file.')
    parser.add_argument('json_file', help='The JSON file to parse.')
    parser.add_argument('source_a', type=int, help='The initial source "a" value.')
    parser.add_argument('source_s', type=int, help='The initial source "s" value.')
    parser.add_argument('depth', type=int, help='The depth of the graph.')
    args = parser.parse_args()

    plot_primitives(args.json_file, args.source_a, args.source_s, args.depth)

if __name__ == "__main__":
    main()
