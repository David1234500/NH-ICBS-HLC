import os
import json
import numpy as np
import matplotlib.pyplot as plt

def load_data_from_directory():
    data = []
    for root, _, files in os.walk('.'):
        if 'config.json' in files and 'results.json' in files:
            with open(os.path.join(root, 'config.json')) as config_file:
                config = json.load(config_file)
            with open(os.path.join(root, 'results.json')) as results_file:
                results = json.load(results_file)
            for result in results:
                if result['feasible']:
                    data.append((result['compute_time'],
                                 config['disc']['dstep'],
                                 config['disc']['hstep']))
    return data

def plot_runtime_vs_dstep(data):
    x, y, _ = zip(*data)

    plt.scatter(x, y, alpha=0.8)
    plt.xlabel('Runtime (ms)')
    plt.ylabel('Dstep (distance per step)')
    plt.savefig("rtvsdstep.png")

def plot_runtime_vs_hstep(data):
    x, _, z = zip(*data)

    plt.scatter(x, z, alpha=0.8)
    plt.xlabel('Runtime (ms)')
    plt.ylabel('Hstep (heading per step)')
    plt.savefig("rtvshstep.png")

if __name__ == '__main__':
    data = load_data_from_directory()
    plot_runtime_vs_dstep(data)
    plot_runtime_vs_hstep(data)
