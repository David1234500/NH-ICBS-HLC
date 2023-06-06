import json
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Rectangle

f = open("mp_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
plt.xlim([0,55])
plt.ylim([-5,20])
print("edges cnt: {}".format(len(y["edges"])))
c = 0

for edge in y["edges"]:
    if edge["source"]["a"] == 0:
        if edge["source"]["s"] == 2:
            
            c = c + 1 
            if c < 20:
                continue

            

            ex = [0]
            ey = [0]

            for curve_point in edge["curve"]:
                ex.append(curve_point["x"])
                ey.append(curve_point["y"])

            # Compute the bounding box
            min_x, max_x = min(ex), max(ex)
            min_y, max_y = min(ey), max(ey)
            ax.add_patch(Rectangle((min_x, min_y), max_x - min_x, max_y - min_y, 
                                   linewidth=2, edgecolor='b', facecolor='none'))

            plt.plot(ex, ey)
            break


plt.title('Motion Primitives with AABB')
plt.xlabel('X Position [cm]')
plt.ylabel('Y Position [cm]')

plt.savefig("../motion_primitives_aabb.png")
