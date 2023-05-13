import json
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def main():
    # Read command-line arguments
    args = sys.argv[1].split(':')
    x, y, a, s = [float(arg) for arg in args]
    dpc = float(sys.argv[2])

    # Load JSON data
    with open('mp_state_graph.json', 'r') as f:
        data = json.load(f)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # Draw motion primitives
    for edge in data['edges']:
        if edge['source']['a'] == a and edge['source']['s'] == s:
            x_offset = x * dpc
            y_offset = y * dpc
            curve_points = [(point['x'] + x_offset, point['y'] + y_offset) for point in edge['curve']]
            ax1.plot(*zip(*curve_points), linewidth=1)
            ax2.plot(*zip(*curve_points), linewidth=1)

    # Draw nodes and highlight motion primitives that leave the convex hull
    nodes = [(node['pose']['x'], node['pose']['y']) for node in data['map']]
    ax2.plot(*zip(*nodes), marker='o', markersize=1, linestyle='None', color='black')

    for edge in data['edges']:
        if edge['source']['a'] == a and edge['source']['s'] == s:
            curve_points = [(point['x'] + x * dpc, point['y'] + y * dpc) for point in edge['curve']]
            ax2.plot(*zip(*curve_points), linewidth=1)

    ax1.set_title('Example Subset of Motion Primitives')  # Added title for ax1
    ax1.set_xlabel('X Position [cm]')  # Added x-axis label for ax1
    ax1.set_ylabel('Y Position [cm]')  # Added y-axis label for ax1
    ax1.set_ylim([-40,40])
    ax1.set_xlim([0,80])

    ax2.set_title('Example Subset of Motion Primitives with 2D State Lattice Projection')  # Added title for ax2
    ax2.set_xlabel('X Position [cm]')  # Added x-axis label for ax2
    ax2.set_ylabel('Y Position [cm]')  # Added y-axis label for ax2
    ax2.set_ylim([-40,40])
    ax2.set_xlim([0,80])


    plt.savefig("../single_mp.png")
    plt.close()

if __name__ == "__main__":
    main()