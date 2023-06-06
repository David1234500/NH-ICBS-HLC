import json
import matplotlib.pyplot as plt
import math

f = open("mp_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
fig = plt.figure(figsize=(12, 8), constrained_layout=True)
# fig, ax = plt.figure()
# for node in y["map"]:
#     plt.plot(node["pose"]["x"],node["pose"]["y"], marker="o")
#     heading = float(node["pose"]["h"])
#     dx = 1 * math.cos(heading)
#     dy = 1 * math.sin(heading)
#     plt.arrow(node["pose"]["x"],node["pose"]["y"], dx, dy)

print("edges cnt: {}".format(len(y["edges"])))

for edge in y["edges"]:
    if edge["source"]["a"] == 1:  #or edge["source"]["a"] == 7 or edge["source"]["a"] == 8:
        if edge["source"]["s"] == 2:

            ex = [0]
            ey = [0]

            for curve_point in edge["curve"]:
                ex.append(curve_point["x"])
                ey.append(curve_point["y"])

            # ex.append(edge["target"]["x"])
            # ey.append(edge["target"]["y"])
            plt.plot(ex,ey)


# plt.tight_layout()

plt.title('Motion Primitives Leaving Central State')  # Added title for ax2
plt.xlabel('X Position [cm]')  # Added x-axis label for ax2
plt.ylabel('Y Position [cm]')  # Added y-axis label for ax2

plt.savefig("../motion_primitives_nc2.png")
