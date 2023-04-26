import json
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def visualize_grid(data, timestamp, heading_a):
    # Extract vehicle 1 start and target positions
    v1_start = data["v1"]["start_pose"]
    v1_target = data["v1"]["target_pose"]

    # Create a grid to display all possible start positions
    grid_size = (90,80)
    grid = np.zeros(grid_size)

    # Color the grid cells based on the given conditions
    grid[v1_start["x"], v1_start["y"]] = 1  # Yellow for vehicle 1 start position
    grid[v1_target["x"], v1_target["y"]] = 1  # Yellow for vehicle 1 target position

    for config in data["v2"]:
        start2 = config["target_pose"]
        print(start2)
        if start2["a"] == heading_a:
            print(start2)
            grid[start2["x"], start2["y"]] = 2  # Red for infeasible start positions with specified heading

    grid[grid == 0] = 3  # Green for the rest of the grid

    # Create a custom color map
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(['yellow', 'red', 'green'])

    # Plot the grid using matplotlib
    plt.imshow(grid, cmap=cmap, origin='lower', extent=[0, grid_size[0], 0, grid_size[1]])
    plt.title(f"Start and Target Positions Visualization for Heading {heading_a}")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.colorbar(ticks=[1, 2, 3], format=plt.FuncFormatter(lambda value, _: {1: 'Yellow', 2: 'Red', 3: 'Green'}[value]))
    
    # Save the figure to the parent directory
    output_filename = f"../dual_vehicles_feasibility_{timestamp}_heading_{heading_a}.png"
    plt.savefig(output_filename)
    plt.close()

# Find all infeasible_configs_... JSON files in the current directory
for json_file in glob.glob("infeasible_configs_*.json"):
    # Load the JSON data
    with open(json_file, "r") as file:
        data = json.load(file)

    # Extract timestamp from the file name
    timestamp = json_file.split("_")[-1].split(".")[0]

    # Visualize the data and save the output for each start heading 'a'
    for heading_a in range(0, 12):
        visualize_grid(data, timestamp, heading_a)
