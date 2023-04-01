import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

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

    def plot_track(track, label):
        for i, pose in enumerate(track[:-1]):
            next_pose = track[i + 1]
            if pose['path_depth_index'] != next_pose['path_depth_index']:
                plt.text(pose['pos'][0], pose['pos'][1], str(pose['path_depth_index']), fontsize=8)

            plt.plot([pose['pos'][0], next_pose['pos'][0]], [pose['pos'][1], next_pose['pos'][1]],
                     color=cmap(pose['path_depth_index'] / max_depth_index), label=label if i == 0 else "")
            plt.scatter(pose['pos'][0], pose['pos'][1], s=20, color=cmap(pose['path_depth_index'] / max_depth_index))

    plot_track(track1, 'Track 1')
    plot_track(track2, 'Track 2')

    for idx, pose in enumerate([pose1, pose2]):
        if idx + 1 in collision_indices:
            plt.scatter(pose['pos'][0], pose['pos'][1], s=100, color=cmap(pose['path_depth_index'] / max_depth_index), alpha=0.5)
            plt.text(pose['pos'][0], pose['pos'][1], f"{idx + 1}, {pose['time_ms']} ms", fontsize=8)

    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Vehicle Tracks and Collisions')
    plt.show()


if __name__ == "__main__":
    prefix = 'collision_in_node'
    files = glob.glob(os.path.join(os.getcwd(), f'{prefix}*'))

    for filename in files:
        plot_tracks(filename, lower_threshold=15)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualize_tracks.py <path_to_json_file>")
    else:
        plot_tracks(sys.argv[1])
