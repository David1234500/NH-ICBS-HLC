import json
import matplotlib.pyplot as plt
import math

f = open("proxy_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
fig, ax = plt.subplots(nrows=y["info"]["size_a"])
index = 0
for f in ax:

    for node in y["map"]:
        f.plot(node["pose"]["x"],node["pose"]["y"])
        # heading = float(node["pose"]["h"])
        # dx = 0.7 * math.cos(heading)
        # dy = 0.7 * math.sin(heading)
        # f.arrow(node["pose"]["x"],node["pose"]["y"], dx, dy)

    print("edges cnt: {}".format(len(y["edges"])))
    for edge in y["edges"]:
        ex = [0]
        ey = [0]
        if edge["source"]["a"] != index:
            continue
        for curve_point in edge["curve"]:
            ex.append(curve_point["x"])
            ey.append(curve_point["y"])

        ex.append(edge["target"]["x"])
        ey.append(edge["target"]["y"])
        f.plot(ex,ey)
    index += 1

# plt.figure(figsize=(8, 8), dpi=80)
plt.show()

