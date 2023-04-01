import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap
import glob
import os

def main(json_file):
    with open(json_file, 'r') as file:
        data = json.load(file)

    # Extract data from JSON
    track1 = data['track1']
    track2 = data['track2']
    pose1 = data['pose1']
    pose2 = data['pose2']
    constraint_node = data['constraint_node']

    # Prepare data for visualization
    t1_x, t1_y, t1_t = zip(*[(p['pos'][0], p['pos'][1], p['time_ms']) for p in track1])
    t2_x, t2_y, t2_t = zip(*[(p['pos'][0], p['pos'][1], p['time_ms']) for p in track2])

    collision_x = [pose1['pos'][0], pose2['pos'][0]]
    collision_y = [pose1['pos'][1], pose2['pos'][1]]

    # Normalize time values for coloring
    norm = Normalize(vmin=min(t1_t + t2_t), vmax=max(t1_t + t2_t))
    cmap = get_cmap('viridis')

    # Calculate distance and time for second and fourth subplots
    distance = [np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) for x1, y1, x2, y2 in zip(t1_x, t1_y, t2_x, t2_y)]
    time = [min(t1, t2) for t1, t2 in zip(t1_t, t2_t)]

    # Create subplots
    fig, axes = plt.subplots(3, 1, figsize=(10, 15))

    # First subplot
    for x, y, t in zip(t1_x, t1_y, t1_t):
        axes[0].scatter(x, y, color=cmap(norm(t)))
    for x, y, t in zip(t2_x, t2_y, t2_t):
        axes[0].scatter(x, y, color=cmap(norm(t)))
    axes[0].scatter(collision_x, collision_y, color='red', marker='o', facecolors='none', s=100, linewidths=1)
    axes[0].set_title('Vehicle Tracks')

    # Second subplot
    axes[1].plot(time, distance)
    axes[1].axvline(pose1['time_ms'], color='red', linestyle='--')
    axes[1].set_title('Distance between Vehicles vs Time')
    axes[1].set_xlabel('Time (ms)')
    axes[1].set_ylabel('Distance')

    # Third subplot
    for x, y, t in zip(t1_x, t1_y, t1_t):
        axes[2].scatter(x, y, color=cmap(norm(t)))
    for x, y, t in zip(t2_x, t2_y, t2_t):
        axes[2].scatter(x, y, color=cmap(norm(t)))
    for avoid in constraint_node['avoid']:
        axes[2].scatter(avoid['x'] * 7, avoid['y'] * 7, color='red', marker='x')
        
    axes[2].scatter(collision_x, collision_y, color='red', marker='o', facecolors='none', s=100, linewidths=1)
    axes[2].set_title('Vehicle Tracks with Constraint Node Information')

    t1_time = [pose['time_ms'] for pose in data['track1']]
    t2_time = [pose['time_ms'] for pose in data['track2']]

#    # Fourth subplot
#     def min_distance_to_constraints(x, y, constraints):
#         min_distance = float('inf')
#         for constraint in constraints:
#             constraint_x = constraint['x'] * 7
#             constraint_y = constraint['y'] * 7
#             distance = np.sqrt((x - constraint_x) ** 2 + (y - constraint_y) ** 2)
#             min_distance = min(min_distance, distance)
#         return min_distance

#     t1_min_constraint_distances = [min_distance_to_constraints(x1, y1, constraint_node['avoid']) for x1, y1, t in zip(t1_x, t1_y, t1_time) if t in time]
#     t2_min_constraint_distances = [min_distance_to_constraints(x2, y2, constraint_node['avoid']) for x2, y2, t in zip(t2_x, t2_y, t2_time) if t in time]

#     axes[3].plot(time, distance, label='Vehicle Distance')
#     axes[3].plot(t1_time, t1_min_constraint_distances, label='Vehicle 1 Min Distance to Constraints')
#     axes[3].plot(t2_time, t2_min_constraint_distances, label='Vehicle 2 Min Distance to Constraints')

#     axes[3].axvline(pose1['time_ms'], color='red', linestyle='--')
#     axes[3].set_title('Vehicle Distances and Min Constraint Node Distance vs Time')
#     axes[3].set_xlabel('Time (ms)')
#     axes[3].set_ylabel('Distance')
#     axes[3].legend()

    
    # Adjust layout and display plot
    plt.tight_layout()
    plt.savefig("../collision_in_" + str(data["constraint_node"]["node_id"]) + ".png")
    # plt.show()

if __name__ == "__main__":
    prefix = 'collision_in_node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        main(filename)

