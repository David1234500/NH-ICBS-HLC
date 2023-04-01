import json
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D


fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')

f = open("mp_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
print("edges cnt: {}".format(len(y["edges"])))

for edge in y["edges"]:
    ex = [0]
    ey = [0]

    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])
    
    ez = [edge["source"]["s"] + edge["targeti"]["s"] for x in ey]

    ax.plot(ex,ey,ez)

plt.savefig("../motion_primitives_3d.svg")