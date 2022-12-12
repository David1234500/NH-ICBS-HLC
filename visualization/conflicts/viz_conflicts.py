import json
import matplotlib.pyplot as plt
import math
import numpy as np
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)



for edge in y["multipath"]["path"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])

    plt.plot(ex,ey)

ax = plt.gca()
for obst in y["avoid"]:
    circ = plt.Circle(( obst["x"] , obst["y"] ), 2 )
    ax.add_artist(circ)

plt.show()

