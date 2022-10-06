import json
import matplotlib.pyplot as plt
import math

f = open("proxy_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
fig, ax = plt.subplots(nrows=y["info"]["size_a"], ncols=y["info"]["size_s"])
r_index = 0
c_index = 0

grid = {}


for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    if(edge["source"]["s"] == 1):
        print("edges from zero velocity: x {} y {}".format(edge["target"]["x"], edge["target"]["y"]))

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])
    ax[edge["source"]["a"]][edge["source"]["s"]].plot(ex,ey)
    ax[edge["source"]["a"]][edge["source"]["s"]].set_xlim([-50,50])
    ax[edge["source"]["a"]][edge["source"]["s"]].set_ylim([-50,50])

print("edges cnt: {}".format(len(y["edges"])))

# plt.figure(figsize=(8, 8), dpi=80)
plt.show()

