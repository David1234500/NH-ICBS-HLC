import json
import matplotlib.pyplot as plt
import math

f = open("proxy_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
for node in y["map"]:
    plt.plot(node["pose"]["x"],node["pose"]["y"], marker="o")
    heading = float(node["pose"]["h"])
    dx = 1 * math.cos(heading)
    dy = 1 * math.sin(heading)
    plt.arrow(node["pose"]["x"],node["pose"]["y"], dx, dy)

print("edges cnt: {}".format(len(y["edges"])))
for edge in y["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])
    plt.plot(ex,ey)

plt.plot(0,0, marker="x")
plt.show()