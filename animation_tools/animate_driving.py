import json
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from PIL import Image
from PIL import ImageOps
from matplotlib.patches import FancyArrowPatch, Circle

def load_cnode_data(filename):
    with open(filename, 'r') as f:
        cnode_data = json.load(f)
    return cnode_data

def add_padding(image, padding):
    return ImageOps.expand(image, border=padding, fill=(0, 0, 0, 0))

def plot_vehicle(ax, vehicle_img, x, y, rotation):
    resized_vehicle_img = vehicle_img.resize((200, 100))
    # padding = int(max(resized_vehicle_img.size) * (np.sqrt(2) - 1) / 2)
    padded_vehicle_img = add_padding(resized_vehicle_img, max(resized_vehicle_img.size))
    vehicle_img_rotated = padded_vehicle_img.rotate(rotation, resample=Image.BICUBIC, expand=False)
    imgplt = ax.imshow(vehicle_img_rotated, zorder=2, extent=[x - 20, x + 20, y - 20, y + 20])


def plot_trajectory(ax, cnode_data):
    for path_vehicle in cnode_data["multipath"]:
        x = [pnode["x"] for pnode in path_vehicle["interprimitive"]]
        y = [pnode["y"] for pnode in path_vehicle["interprimitive"]]
        ax.plot(x, y, linestyle='--', color='gray', zorder=1, linewidth=2.0)

def plot_start_end_positions(ax, cnode_data):
    for path_vehicle in cnode_data["multipath"]:
        start = path_vehicle["interprimitive"][0]
        end = path_vehicle["interprimitive"][-1]

        arrow = FancyArrowPatch((start["x"], start["y"]), (start["x"] + 15 * np.cos(start["a"]), start["y"] + 15 * np.sin(start["a"])),
                                arrowstyle='->', mutation_scale=15, lw=2, color='blue', zorder=3)
        ax.add_patch(arrow)

        circle = Circle((end["x"], end["y"]), radius=4, color='blue', zorder=3)
        ax.add_patch(circle)

def plot_constraints(ax, cnode_data):
    if "avoid" in cnode_data:
        for constraint in cnode_data["avoid"]:
            ax.add_artist(plt.Circle((constraint["x"], constraint["y"]), 3, color='orange', zorder=0))

def min_distance_exceeded(cnode_data, t):
    min_safety_distance = 15
    positions = [pnode for path_vehicle in cnode_data["multipath"] for pnode in path_vehicle["interprimitive"] if pnode["t"] == t]
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            distance = np.sqrt((positions[i]["x"] - positions[j]["x"])**2 + (positions[i]["y"] - positions[j]["y"])**2)
            if distance < min_safety_distance:
                return True
    return False

def interpolate_positions(cnode_data, num_interpolations=3):
    for path_vehicle in cnode_data["multipath"]:
        new_interprimitives = []

        for i in range(len(path_vehicle["interprimitive"]) - 1):
            p1 = path_vehicle["interprimitive"][i]
            p2 = path_vehicle["interprimitive"][i + 1]

            new_interprimitives.append(p1)

            for k in range(1, num_interpolations + 1):
                t = (p1["t"] * (num_interpolations - k + 1) + p2["t"] * k) / (num_interpolations + 1)
                x = (p1["x"] * (num_interpolations - k + 1) + p2["x"] * k) / (num_interpolations + 1)
                y = (p1["y"] * (num_interpolations - k + 1) + p2["y"] * k) / (num_interpolations + 1)
                a = (p1["a"] * (num_interpolations - k + 1) + p2["a"] * k) / (num_interpolations + 1)

                new_interprimitives.append({"t": t, "x": x, "y": y, "a": a})

        new_interprimitives.append(path_vehicle["interprimitive"][-1])
        path_vehicle["interprimitive"] = new_interprimitives

def generate_frames(cnode_data, output_folder, vehicle_img_path):
    os.makedirs(output_folder, exist_ok=True)

    vehicle_img = Image.open(vehicle_img_path)

    max_time = int(max([pnode["t"] for path_vehicle in cnode_data["multipath"] for pnode in path_vehicle["interprimitive"]]))
    index = 0

    for t in range(max_time + 1):
        if min_distance_exceeded(cnode_data, t):
            break

        fig, ax = plt.subplots(figsize=(15, 15))
        ax.set_xlim(0, 250)
        ax.set_ylim(0, 250)
        ax.set_aspect('equal')

        ax.set_xlabel("X Position [cm]")
        ax.set_ylabel("Y Position [cm]")

        ax.set_xticks(np.arange(0, 250, 25))
        ax.set_yticks(np.arange(0, 250, 25))

        plot_constraints(ax, cnode_data)
        plot_trajectory(ax, cnode_data)
        plot_start_end_positions(ax, cnode_data)

        plot = False
        for path_vehicle in cnode_data["multipath"]:
            for pnode in path_vehicle["interprimitive"]:
                if pnode["t"] == t:
                    x, y = pnode["x"], pnode["y"]
                    rotation = np.rad2deg(pnode["a"])
                    plot_vehicle(ax, vehicle_img, x, y, rotation)
                    plot = True

        if plot:
            plt.savefig(os.path.join(output_folder, f'frame_{index:04d}.png'))
            index += 1
            plt.close()

if __name__ == "__main__":

    # mpl.rcParams.update({'font.size': 50})

    cnode_data = load_cnode_data('node3.json')
    interpolate_positions(cnode_data, num_interpolations=8)
    generate_frames(cnode_data, 'video_frames', 'vehicle.png')