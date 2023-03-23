import json
import matplotlib.pyplot as plt
import math

f = open("mp_state_graph.json", "r")
x = f.read() 
y = json.loads(x)
  
for node in y["map"]:
    plt.plot(node["pose"]["x"],node["pose"]["y"], marker="o")
    heading = float(node["pose"]["h"])
    dx = 2 * math.cos(heading)
    dy = 2 * math.sin(heading)
    plt.arrow(node["pose"]["x"],node["pose"]["y"], dx, dy)

plt.plot(0,0, marker="x")
plt.show()