import json
import matplotlib.pyplot as plt

with open("mp_state_graph.json", "r") as f:
    data = json.load(f)

fig = plt.figure(figsize=(60, 60), constrained_layout=True)
fig.suptitle('Motion Primitives Visualisation')
grid = {}

velocity_combinations = set()
for edge in data["edges"]:
    velocity_combinations.add((2 * edge["source"]["s"] + edge["targeti"]["s"], edge["source"]["a"]))
velocity_combinations = sorted(list(velocity_combinations))

subfigs = fig.subfigures(nrows=len(velocity_combinations), ncols=1)
for index, (source_velocity, target_velocity) in enumerate(velocity_combinations):
    subfig = subfigs[index]
    subfig.suptitle(f'Source Velocity Level: {source_velocity}')
    grid[source_velocity] = {}

    axs = subfig.subplots(nrows=1, ncols=data["info"]["size_a"])
    for col, ax in enumerate(axs):
        ax.plot()
        ax.set_title(f'S Heading: {col}')
        grid[source_velocity][col] = ax

for edge in data["edges"]:
    ex = [0]
    ey = [0]
    for curve_point in edge["curve"]:
        ex.append(curve_point["x"])
        ey.append(curve_point["y"])

    ex.append(edge["target"]["x"])
    ey.append(edge["target"]["y"])

    source_velocity = 2 * edge["source"]["s"] + edge["targeti"]["s"]
    ax = grid[source_velocity][edge["source"]["a"]]
    ax.set_aspect('equal')
    ax.plot(ex, ey)
    ax.set_xlim([-100, 100])
    ax.set_xlabel("cm")
    ax.set_ylabel("cm")
    ax.set_ylim([-100, 100])

plt.savefig("../motion_primitives_by_angle_speed.svg", dpi=500)
