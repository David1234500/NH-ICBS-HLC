import os
import json
import glob
import matplotlib.pyplot as plt

def load_json_files(prefix):
    files = glob.glob(f"{prefix}*.json")
    paths = []

    for file in files:
        with open(file, 'r') as f:
            data = json.load(f)
            paths.append((file, data['path']))

    return paths

def plot_paths(paths):
    plt.figure()

    for file, path in paths:
        x = [point['x'] for point in path]
        y = [point['y'] for point in path]
        plt.plot(x, y, label=file)

    plt.legend()
    plt.savefig("../simple_path.png")

if __name__ == "__main__":
    prefix = "mpnlopt_single_test_"  
    paths = load_json_files(prefix)
    plot_paths(paths)
