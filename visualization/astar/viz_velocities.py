import json
import matplotlib.pyplot as plt
import sys

def visualize_path_data(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(10, 15))

    # Subplot 1: Path visualization
    path_x = [point['x'] for point in data['interprimitive']]
    path_y = [point['y'] for point in data['interprimitive']]
    base_nodes_x = [point['bnode']['x'] for point in data['interprimitive']]
    base_nodes_y = [point['bnode']['y'] for point in data['interprimitive']]
    
    axs[0].plot(path_x, path_y, 'b', label='Path')
    axs[0].scatter(base_nodes_x, base_nodes_y, c='r', label='Base Nodes')
    axs[0].set_title('Path and Base Nodes')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].legend()

    # Subplot 2: Heading over time
    time = [point['t'] for point in data['interprimitive']]
    heading = [point['a'] for point in data['interprimitive']]

    axs[1].plot(time, heading, 'g')
    axs[1].set_title('Heading over Time')
    axs[1].set_xlabel('Time (ms)')
    axs[1].set_ylabel('Heading (rad)')

    # Subplot 3: Velocity over time
    velocity = [point['s'] for point in data['interprimitive']]

    axs[2].plot(time, velocity, 'm')
    axs[2].set_title('Velocity over Time')
    axs[2].set_xlabel('Time (ms)')
    axs[2].set_ylabel('Velocity (m/s)')

    # Save the plot
    fig.savefig('../path_velocities.png')
    plt.close(fig)


if __name__ == '__main__':
    json_file = sys.argv[1]
    visualize_path_data(json_file)
