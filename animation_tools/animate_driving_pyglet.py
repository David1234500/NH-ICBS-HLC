import json
import os
import numpy as np
from PIL import Image
from PIL import ImageOps
import pyglet
from pyglet.gl import glClearColor
from pyglet.image.codecs.ffmpeg import FFmpegEncoder

def load_cnode_data(filename):
    with open(filename, 'r') as f:
        cnode_data = json.load(f)
    return cnode_data

def add_padding(image, padding):
    return ImageOps.expand(image, border=padding, fill=(0, 0, 0, 0))

def min_distance_exceeded(cnode_data, t):
    min_safety_distance = 15
    positions = [pnode for path_vehicle in cnode_data["multipath"] for pnode in path_vehicle["interprimitive"] if pnode["t"] == t]
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            distance = np.sqrt((positions[i]["x"] - positions[j]["x"])**2 + (positions[i]["y"] - positions[j]["y"])**2)
            if distance < min_safety_distance:
                return True
    return False

def generate_frames(cnode_data, output_folder, vehicle_img_path):
    os.makedirs(output_folder, exist_ok=True)

    vehicle_img = Image.open(vehicle_img_path)

    max_time = int(max([pnode["t"] for path_vehicle in cnode_data["multipath"] for pnode in path_vehicle["interprimitive"]]))

    window = pyglet.window.Window(width=600, height=600, visible=False)
    glClearColor(1, 1, 1, 1)

    encoder = FFmpegEncoder('output_animation.mp4')

    for t in range(max_time + 1):
        if min_distance_exceeded(cnode_data, t):
            break

        window.clear()
        window.dispatch_events()

        # Draw vehicle images
        for path_vehicle in cnode_data["multipath"]:
            for pnode in path_vehicle["interprimitive"]:
                if pnode["t"] == t:
                    x, y = pnode["x"], pnode["y"]
                    rotation = -np.rad2deg(pnode["a"])
                    padded_vehicle_img = add_padding(vehicle_img, max(vehicle_img.size))
                    vehicle_img_rotated = padded_vehicle_img.rotate(rotation, resample=Image.BICUBIC, expand=True)
                    vehicle_img_pyglet = pyglet.image.create(vehicle_img_rotated.width, vehicle_img_rotated.height)
                    vehicle_img_pyglet.set_data('RGBA', vehicle_img_rotated.width * 4, vehicle_img_rotated.tobytes())
                    vehicle_img_pyglet.blit(x - vehicle_img_rotated.size[0] / 2, y - vehicle_img_rotated.size[1] / 2)

        # Capture frame and add to the encoder
        frame_buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        frame_data = frame_buffer.get_image_data()
        encoder.add_frame(frame_data)

    encoder.close()

if __name__ == "__main__":
    cnode_data = load_cnode_data('node0.json')
    generate_frames(cnode_data, 'video_frames', 'vehicle.png')
