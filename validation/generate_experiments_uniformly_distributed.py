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
    heading = random.randint(0, 12)
    velocity = 1
    return {"x": x, "y": y, "a": heading, "s": velocity, "t":0}

def generate_experiments(rectangle, output_filename, experiment_number, vehicle_count, min_distance=6):
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
parser.add_argument("x_min", type=int, help="Minimum x-coordinate of the rectangle")
parser.add_argument("x_max", type=int, help="Maximum x-coordinate of the rectangle")
parser.add_argument("y_min", type=int, help="Minimum y-coordinate of the rectangle")
parser.add_argument("y_max", type=int, help="Maximum y-coordinate of the rectangle")
parser.add_argument("output_filename", help="Output file to store generated experiments")
parser.add_argument("experiment_number", type=int, help="Number of experiments to generate")
parser.add_argument("vehicle_count", type=int, help="Number of vehicles for each experiment")

# Parse command-line arguments
args = parser.parse_args()

# Define rectangle using parsed arguments
rectangle = {
    "x_min": args.x_min,
    "x_max": args.x_max,
    "y_min": args.y_min,
    "y_max": args.y_max,
}

# Call generate_experiments function with specified arguments
generate_experiments(rectangle, args.output_filename, args.experiment_number, args.vehicle_count)
