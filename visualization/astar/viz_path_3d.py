import json
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_interprimitive_data(filename):
    with open(filename, 'r') as f:
        data = json.load(f)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_values = [p['x'] for p in data['interprimitive']]
    y_values = [p['y'] for p in data['interprimitive']]
    heading_values = [p['a'] for p in data['interprimitive']]
    velocity_values = [p['s'] for p in data['interprimitive']]

    sc = ax.scatter(x_values, y_values, heading_values, c=velocity_values, cmap='viridis')

    # Set axis labels
    ax.set_xlabel('X Position [cm]')
    ax.set_ylabel('Y Position [cm]')
    ax.set_zlabel('Heading [rad]')

    # Add a colorbar to represent the velocity dimension
    cbar = fig.colorbar(sc, ax=ax, label='Velocity')

    plt.savefig("../interprimitive_data_3d.png")
    plt.show()

if __name__ == "__main__":
    # Replace "data.json" with the name of your JSON file
    visualize_interprimitive_data(sys.argv[1])
