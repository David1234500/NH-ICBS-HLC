import json
import random
import math
import argparse

def distance(point1, point2):
    return math.sqrt((point1["x"] - point2["x"])**2 + (point1["y"] - point2["y"])**2)

def all_distances_sufficient(points, min_distance):
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if distance(points[i], points[j]) < min_distance:
                return False
    return True

def generate_random_point(rectangle):
    x = random.randint(rectangle["x_min"], rectangle["x_max"])
    y = random.randint(rectangle["y_min"], rectangle["y_max"])
    heading = random.randint(0, 11)
    velocity = 2
    return {"x": x, "y": y, "a": heading, "s": velocity, "t":0}

def generate_experiments(rectangle, output_filename, experiment_number, vehicle_count, min_distance=7):
    experiments = []

    for _ in range(experiment_number):
        while True:
            selected_positions = [generate_random_point(rectangle) for _ in range(2 * vehicle_count)]

            if all_distances_sufficient(selected_positions, min_distance):
                break

        experiment = {
            "starts": selected_positions[:vehicle_count],
            "targets": selected_positions[vehicle_count:]
        }
        experiments.append(experiment)

    with open(output_filename, "w") as output_file:
        json.dump(experiments, output_file, indent=4)

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Generate experiments for vehicle routing.")
parser.add_argument("input_filename", help="Input file to load the rectangle coordinates from")
parser.add_argument("output_filename", help="Output file to store generated experiments")
parser.add_argument("experiment_number", type=int, help="Number of experiments to generate")
parser.add_argument("vehicle_count", type=int, help="Number of vehicles for each experiment")

# Parse command-line arguments
args = parser.parse_args()

# Load rectangle coordinates from input file
with open(args.input_filename, "r") as input_file:
    rectangle = json.load(input_file)

# Call generate_experiments function with specified arguments
generate_experiments(rectangle, args.output_filename, args.experiment_number, args.vehicle_count)
