import json
import matplotlib.pyplot as plt
import math

f = open("proxy_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
fig, ax = plt.subplots(nrows= int(y["info"]["size_a"]/4) + 1, ncols=3 * y["info"]["size_s"])
r_index = 0
c_index = 0

grid = {}


for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["t_pose"]["x"])
    ey.append(edge["t_pose"]["y"])

    ax[edge["source"]["a"]][int(3 * edge["source"]["s"] + edge["t_index"]["s"])].plot(ex,ey)
    ax[edge["source"]["a"]][int(3 * edge["source"]["s"] + edge["t_index"]["s"])].set_xlim([-50,50])
    ax[edge["source"]["a"]][int(3 * edge["source"]["s"] + edge["t_index"]["s"])].set_ylim([-50,50])

print("edges cnt: {}".format(len(y["edges"])))

# plt.figure(figsize=(8, 8), dpi=80)
plt.show()

