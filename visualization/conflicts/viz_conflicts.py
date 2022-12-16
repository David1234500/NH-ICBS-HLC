import json
import matplotlib.pyplot as plt
import math
import numpy as np
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)

for pathe in y["multipath"]:
    
        ex = [0]
        ey = [0]
        for edge in pathe["path"]:
            ex.append(edge["x"])
            ey.append(edge["y"])

        plt.plot(ex,ey)

ax = plt.gca()
for obst in y["avoid"]:
    circ = plt.Circle(( obst["x"] , obst["y"] ), 2 )
    ax.add_artist(circ)

plt.show()

