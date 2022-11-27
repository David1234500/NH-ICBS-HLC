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


experiments = sorted(experiments, key= lambda x: x["settings"]["node_count"])

for timstep_size in range(300,700,100):
    for quality_level in range(2,5):
        mean_path_length = []
        mean_time_length = []
        edge_count = []
        node_count = []

        for e in experiments:
            
            if e["settings"]["quality"] == quality_level and timstep_size == e["settings"]["test_timestep"]:
                if e["result"]["reachability"]:
                    edge_count.append(e["result"]["edge_count"])
                    node_count.append(e["settings"]["node_count"])
                    mean_path_length.append(e["result"]["mean_path_length"])
                    mean_time_length.append(e["result"]["mean_time_length"])


    
            
        fig, ax1 = plt.subplots()

        ax2 = ax1.twinx()
        ax1.plot(node_count, mean_path_length, 'g-')
        ax2.plot(node_count, edge_count, 'b-')

        ax1.set_xlabel('Node Count (Proxy for Discretisation Resolution)')
        ax1.set_ylabel('Mean Path Length to reach neighborhood', color='g')
        ax2.set_ylabel('Edge Count', color='b')

        # plt.suptitle("State Space Eval with Q{}.png".format(quality_level), fontsize=7)
        plt.savefig("f2_quality_timestep_{}_demo_{}.png".format(timstep_size, quality_level))
        plt.clf()