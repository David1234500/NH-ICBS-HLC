import json
import sys
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def visualize_json(filename):
    with open(filename, 'r') as f:
        data = json.load(f)

    fig, ax = plt.subplots(figsize=(12, 6))

    

    for i, interprimitive in enumerate(data['interprimitive']):
        bnode = interprimitive['bnode']

        # Draw unused motion primitives in grey, excluding the last set
        if 'unused_primitives' in bnode and i < len(data['interprimitive']) - 1:
            for primitive in bnode['unused_primitives']:
                primitive_path = [(point['x'], point['y']) for point in primitive['mp_path']]
                ax.plot(*zip(*primitive_path), color='grey', linewidth=0.5, alpha=0.5)

    # Set axis labels
    ax.set_xlabel('X Position [cm]')
    ax.set_ylabel('Y Position [cm]')

    # Draw interprimitive positions and unused motion primitives
    interprimitive_positions = [(interprimitive['x'], interprimitive['y']) for interprimitive in data['interprimitive']]
    ax.plot(*zip(*interprimitive_positions), color='blue', linewidth=2)

    # Add legend for grey and blue MPs
    legend_elements = [
        Line2D([0], [0], color='blue', lw=2, label='Used Motion Primitives'),
        Line2D([0], [0], color='grey', lw=0.5, alpha=0.5, label='Unused but accessible Motion Primitives'),
    ]
    ax.legend(handles=legend_elements, loc='best')

    plt.savefig("../path_with_mps.png")
    plt.show()

if __name__ == "__main__":
    # Replace "data.json" with the name of your JSON file
    visualize_json(sys.argv[1])
