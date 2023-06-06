import os
import json
import matplotlib.pyplot as plt

def load_json_files(directory):
    json_files = []
    for file in os.listdir(directory):
        if file.endswith('.json') and file.startswith('hull_'):
            with open(os.path.join(directory, file), 'r') as f:
                json_data = json.load(f)
                json_files.append(json_data)
    return json_files

def visualize_hulls(json_files):
    num_files = len(json_files)
    fig, axes = plt.subplots(nrows=num_files, ncols=1, figsize=(8, num_files * 6))
    
    if num_files == 1:
        axes = [axes]

    for i, data in enumerate(json_files):
        source_velocity = data['source_velocity']
        target_velocity = data['target_velocity']
        heading = data['heading']
        
        # if source_velocity == 2 and target_velocity == 2 and heading == 1:
       
        print(data.keys())
        hull_points = [(5 * point['x'], 5 * point['y']) for point in data['points']]

        x, y = zip(*hull_points)
        axes[i].scatter(x, y, s=1)
        axes[i].set_title(f'Canidate States for Primitive Computation')
        axes[i].set_xlabel("Position X[cm]")
        axes[i].set_ylabel("Position Y[cm]")
        axes[i].set_aspect('equal', adjustable='datalim')

    # plt.tight_layout()
    plt.savefig("../conic_hulls.png")

if __name__ == '__main__':
    json_files = load_json_files(".")
    visualize_hulls(json_files)
