import json
import matplotlib.pyplot as plt
import math
import sys

def to_pose_list(json_pose_data):
    pose_data = {}
    for data_vehicle in json_pose_data:
        vehicle_id = data_vehicle["vehicle"]
        pose_data[vehicle_id] = []
        for pose in data_vehicle["path"]:
            pose_data[vehicle_id].append(pose)

    return pose_data

f_actual = open("actual_trajectories.json", "r")
x_actual = f_actual.read() 
actual_pose_data = json.loads(x_actual)
  
f_reference = open("reference_trajectories.json", "r")
x_reference = f_reference.read() 
reference_pose_data = json.loads(x_reference)

actual_poses_by_vehicle = to_pose_list(actual_pose_data)
reference_poses_by_vehicle = to_pose_list(reference_pose_data)
    
for v in actual_pose_data:
    
    plt.plot(actual_pose_data[v][:]["time_ms"],actual_pose_data[v][:]["pv"], marker="x")
    plt.plot(reference_poses_by_vehicle[v][:]["time_ms"],reference_poses_by_vehicle[v][:]["pv"], marker="x")
    
    plt.xlabel("Time ms")
    plt.ylabel("Velocity m/s")

    plt.suptitle("vel_comparison_{}.png".format(v), fontsize=7)
    plt.savefig("vel_comparison_{}.png".format(v))
    plt.clf()