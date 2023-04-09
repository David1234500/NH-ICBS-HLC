import json
import random

def generate_experiments(input_filename, output_filename, experiment_number):
    with open(input_filename) as input_file:
        data = json.load(input_file)

    if len(data) < 8:
        raise ValueError("Input file should have at least 8 positions")

    experiments = []

    for _ in range(experiment_number):
        selected_positions = random.sample(data, 8)
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
