import json
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

def plot_visited_nodes(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    start = data['start']
    target = data['target']
    nodes = data['nodes']

    node_coords = np.array([[node['x'], node['y']] for node in nodes])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    ax1.scatter(start['x'], start['y'], c='g', marker='o', label='Start')
    ax1.scatter(target['x'], target['y'], c='r', marker='o', label='Target')

    if len(node_coords) > 2:
        hull = ConvexHull(node_coords)
        for simplex in hull.simplices:
            ax1.plot(node_coords[simplex, 0], node_coords[simplex, 1], 'k-', alpha=0.5)

    ax1.set_title('Start, Target, and Hull of Visited Nodes')
    ax1.legend()

    ax2.scatter(node_coords[:, 0], node_coords[:, 1], c='b', marker='o', alpha=0.5)
    ax2.set_title('All Visited Nodes')

    plt.show()

if __name__ == '__main__':
    json_file = sys.argv[1]
    plot_visited_nodes(json_file)
