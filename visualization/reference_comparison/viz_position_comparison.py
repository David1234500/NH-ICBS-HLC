import json
import matplotlib.pyplot as plt
import math
import sys
import numpy as np


f_actual = open(sys.argv[1], "r")
x_actual = f_actual.read() 
pose_data = json.loads(x_actual)
  
time_ref_ms = pose_data["time_ref_ms"]

actual_data = []
for node in pose_data["actual"][0]["path"]:
    if node["time_ms"] < 1000000:
        actual_data.append((node["px"],node["py"],node["ph"],node["pv"],node["time_ms"]))

ref_data = []
for node in pose_data["ref"][0]["path"]:
    ref_data.append((node["px"],node["py"],node["ph"],node["pv"],node["time_ms"]))

# print(actual_data)
# print(ref_data)

fig = plt.figure(constrained_layout=True)
fig.suptitle('Motion Primitives Visualisation')

# create 3x1 subfigs
subfigs = fig.subplots(nrows=4, ncols=1)
ref_data_np = np.array(ref_data)
actual_data_np = np.array(actual_data)

# plot velocity
times = ref_data_np[:,4]
speeds = ref_data_np[:,3]
subfigs[0].plot(times,speeds)

times2 = actual_data_np[:,4] 
speeds2 = actual_data_np[:,3] 
subfigs[0].plot(times2,speeds2)

# plot heading
times = ref_data_np[:,4]
speeds = ref_data_np[:,2] - 2 * math.pi
subfigs[1].plot(times,speeds)

times2 = actual_data_np[:,4] 
speeds2 = actual_data_np[:,2] 
subfigs[1].plot(times2,speeds2)

# plot position x
times = ref_data_np[:,4]
speeds = ref_data_np[:,1] 
subfigs[2].plot(times,speeds)

times2 = actual_data_np[:,4] 
speeds2 = actual_data_np[:,1] 
subfigs[2].plot(times2,speeds2)

# plot position y
times = ref_data_np[:,4]
speeds = ref_data_np[:,0] 
subfigs[3].plot(times,speeds)

times2 = actual_data_np[:,4] 
speeds2 = actual_data_np[:,0] 
subfigs[3].plot(times2,speeds2)

plt.show()