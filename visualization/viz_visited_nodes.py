import json
import matplotlib.pyplot as plt
import math

'''  visited_nodes["target"]["x"] = target.x;
    visited_nodes["target"]["y"] = target.y;
    visited_nodes["target"]["a"] = target.a;
    visited_nodes["target"]["s"] = target.s;

    visited_nodes["start"]["x"] = start.x;
    visited_nodes["start"]["y"] = start.y;
    visited_nodes["start"]["a"] = start.a;
    visited_nodes["start"]["s"] = start.s;

    for(auto it: cameFrom){

        json node;
        node["x"] = it.second.x;
        node["y"] = it.second.y;
        node["a"] = it.second.a;
        node["s"] = it.second.s;

        visited_nodes["nodes"].push_back(node);
    } '''



f = open("visited_nodes.json", "r")
x = f.read() 
y = json.loads(x)
  
plt.plot(y["start"]["x"],y["start"]["y"], marker="o")
plt.plot(y["target"]["x"],y["target"]["y"], marker="x")

plt.xlim((0,90))
plt.ylim((0,90))

for v in y["nodes"]:
    x = [v["x"],v["px"]]
    y = [v["y"],v["py"]]

    plt.plot(x,y)
    plt.plot(v["x"],v["y"], marker="1")

plt.show()