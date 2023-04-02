import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import glob
import os

def plot_tracks(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    track1 = data['track1']
    track2 = data['track2']
    pose1 = data['pose1']
    pose2 = data['pose2']
    collision_indices = [data['index1'], data['index2']]

    cmap = plt.cm.get_cmap('viridis')
    max_depth_index = max(max([pose['path_depth_index'] for pose in track1]), max([pose['path_depth_index'] for pose in track2]))

    def plot_track(track, label, id):
        for i, pose in enumerate(track[:-1]):
            next_pose = track[i + 1]
            if pose['path_depth_index'] != next_pose['path_depth_index']:
                plt.text(pose['pos'][0], pose['pos'][1], str(id) + ":" + str(pose['path_depth_index']), fontsize=8)

            plt.plot([pose['pos'][0], next_pose['pos'][0]], [pose['pos'][1], next_pose['pos'][1]],
                     color=cmap(pose['path_depth_index'] / max_depth_index), label=label if i == 0 else "")
            plt.scatter(pose['pos'][0], pose['pos'][1], s=20, color=cmap(pose['path_depth_index'] / max_depth_index))

    plot_track(track1, f"Track 1 {data['car_index_1']} f:{data['feasible1']} track end:{data['track_end']}", 1)
    plot_track(track2, f"Track 2 {data['car_index_2']} f:{data['feasible2']} track end:{data['track_end']}", 2)

    for idx, pose in enumerate([pose1, pose2]):
            plt.scatter(pose['pos'][0], pose['pos'][1], s=250, color="red", alpha=0.5)
            plt.text(pose['pos'][0], pose['pos'][1], f" {pose['time_ms']} ms", fontsize=8)

    plt.scatter(data['pose1bi']['x'] * 7, data['pose1bi']['y'] * 7, s=250, color="orange", alpha=0.5)
    plt.text(data['pose1bi']['x'] * 7, data['pose1bi']['y'] * 7, f" {data['index1']} r: {data['car_rem_index_1']}", fontsize=13)
    plt.scatter(data['pose2bi']['x'] * 7, data['pose2bi']['y'] * 7, s=250, color="orange", alpha=0.5)
    plt.text(data['pose2bi']['x'] * 7, data['pose2bi']['y'] * 7, f" {data['index2']} r: {data['car_rem_index_2']}", fontsize=13)

    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Vehicle Tracks and Collisions')
    plt.savefig("../pdepth_" + str(data["constraint_node"]["node_id"]) + ".png")
    plt.close()

if __name__ == "__main__":
    prefix = 'collision_in_node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        plot_tracks(filename)
