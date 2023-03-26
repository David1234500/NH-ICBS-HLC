import json
import matplotlib.pyplot as plt
import math
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)
  
ex = []
ey = []



for node in y["path"]:
    ex.append(node["x"])
    ey.append(node["y"])


plt.plot(ex,ey)

# plt.plot(0,0, marker="x")
plt.show()