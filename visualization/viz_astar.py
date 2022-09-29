import json
import matplotlib.pyplot as plt
import math
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)
  
ex = []
ey = []

plt.xlim([0,400])
plt.ylim([0,400])

for node in y["path"]:
    ex.append(node["x"])
    ey.append(node["y"])


plt.xlim([0, 400])
plt.ylim([0, 400])
plt.plot(ex,ey)

# plt.plot(0,0, marker="x")
plt.show()