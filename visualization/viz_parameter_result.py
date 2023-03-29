import json
import matplotlib.pyplot as plt

def visualize_data(file_name):
    with open(file_name, 'r') as f:
        data = json.load(f)

    edges = data['edges']
    map_points = data['map']

    # Plot the edges
    for edge in edges:
        x_coords = [point['x'] for point in edge['edge']['curve']]
        y_coords = [point['y'] for point in edge['edge']['curve']]
        plt.plot(x_coords, y_coords, label=edge['ID'])

    # Plot the map points
    map_x_coords = [point['x'] for point in map_points]
    map_y_coords = [point['y'] for point in map_points]
    plt.scatter(map_x_coords, map_y_coords, marker='o', color='red')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Vehicle Trajectories and Map Points')
    plt.legend()
    plt.grid(True)
    plt.savefig("../parameter_result.png")

# Call the function with the file name
visualize_data('mp_param_res.json')
