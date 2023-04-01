import json
import matplotlib.pyplot as plt
import math
import sys
import os

experiments = []

for filename in os.listdir(os.getcwd()):
    if "_info.json" in filename:
        with open(os.path.join(os.getcwd(), filename), 'r') as f:
            x = f.read() 
            y = json.loads(x)
            experiments.append(y)

for quality_level in range(2,5):
    for e in experiments:
        if e["settings"]["quality"] == quality_level:
            node_count = e["settings"]["node_count"]
            test_timestep = e["settings"]["test_timestep"]

            reachable = e["result"]["reachability"]
            mean_path_length = e["result"]["mean_path_length"]
            
            if not reachable:
                plt.plot(node_count, test_timestep, marker="x",color=[0.0, 0.0, 1.0])
            else:
                plt.plot(node_count, test_timestep, marker="o",color=[1.0, 0, 0])

    plt.xlabel("Node Count")
    plt.ylabel("Timestep size")
    plt.suptitle("State Space Eval with Q{}.svg".format(quality_level), fontsize=7)
    plt.savefig("quality_demo_{}.svg".format(quality_level))
    plt.clf()