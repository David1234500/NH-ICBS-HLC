import json
import matplotlib.pyplot as plt
import math
import numpy as np
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)

ax = plt.gca()

for pathe in y["multipath"]:
    
        ex = []
        ey = []

        nx = []
        ny = []

        for edge in pathe["path"]:
            nx.append(edge["x"])
            ny.append(edge["y"])

        for edge in pathe["interprimitive"]:
            ex.append(edge["x"])
            ey.append(edge["y"])

        plt.plot(ex,ey)
        plt.scatter(nx,ny)


if "avoid" in y:
    for obst in y["avoid"]:
        circ = plt.Circle(( obst["x"] , obst["y"] ), 2 )
        ax.add_artist(circ)


plt.show()

