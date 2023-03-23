
import json
import matplotlib.pyplot as plt
import math
import numpy as np
import sys


f = open(sys.argv[1], "r")
x = f.read() 
y = json.loads(x)

ax = plt.gca()

#   pnode["xa"] = pa.at(i).pos[0];
#         pnode["ya"] = pa.at(i).pos[1];
#         pnode["ta"] = pa.at(i).time_ms;

#         pnode["xb"] = pb.at(i).pos[0];
#         pnode["yb"] = pb.at(i).pos[1];
#         pnode["tb"] = pb.at(i).time_ms;

#     pnode["d"] = comp_pose_dist.at(i);

collisions = []

for check in y["conflicts"]:    
    collisions.append((check["xa"],check["ya"],check["xb"],check["yb"]))
    
    x = [check["xa"],check["xb"]]
    y = [check["ya"],check["yb"]]
    
    plt.plot(x,y)

np_collision = np.array(collisions)
print(np_collision)
plt.scatter(np_collision[:,0],np_collision[:,1])
plt.scatter(np_collision[:,2],np_collision[:,3])


plt.show()

