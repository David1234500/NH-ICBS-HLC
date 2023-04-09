import json
import random
import math

def distance(point1, point2):
    return math.sqrt((point1["x"] - point2["x"])**2 + (point1["y"] - point2["y"])**2)

def all_distances_sufficient(points, min_distance):
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if distance(points[i], points[j]) < min_distance:
                return False
    return True

def generate_experiments(input_filename, output_filename, experiment_number, min_distance=4):
    with open(input_filename) as input_file:
        data = json.load(input_file)

    if len(data) < 8:
        raise ValueError("Input file should have at least 8 positions")

    experiments = []

    for _ in range(experiment_number):
        while True:
            selected_positions = random.sample(data, 8)
            if all_distances_sufficient(selected_positions, min_distance):
                break

        experiment = {
            "starts": selected_positions[:4],
            "targets": selected_positions[4:]
        }
        experiments.append(experiment)

    with open(output_filename, "w") as output_file:
        json.dump(experiments, output_file, indent=4)

input_filename = "pose_data.json"
output_filename = "experiments.json"
experiment_number = 20

generate_experiments(input_filename, output_filename, experiment_number)
