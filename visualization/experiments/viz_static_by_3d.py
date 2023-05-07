import os
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import Rbf

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

def create_3d_plot(data):
    fig = plt.figure(figsize=(15,15))
    ax = fig.add_subplot(111, projection='3d')

    z, y, x = zip(*data)

    xi = np.linspace(min(x), max(x), 200)
    yi = np.linspace(min(y), max(y), 200)
    X, Y = np.meshgrid(xi, yi)
    
    rbf = Rbf(x, y, z, function='multiquadric', smooth=0.5)
    Z = rbf(X, Y)

    ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)

    ax.set_zlabel('Runtime (ms)')
    ax.set_ylabel('Dstep (distance per step)')
    ax.set_xlabel('Hstep (heading per step)')

    plt.savefig("gridplot.png")

if __name__ == '__main__':
    data = load_data_from_directory()
    create_3d_plot(data)
