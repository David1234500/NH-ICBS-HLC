import json
import random
import math
import argparse

def distance(point1, point2):
    return math.sqrt((point1["x"] - point2["x"])**2 + (point1["y"] - point2["y"])**2)

def all_distances_exact(points, min_distance):
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if distance(points[i], points[j]) == min_distance:
                return False
    return True

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

def point_in_restrictions(point, restrictions, restriction_key):
    for restriction in restrictions:
        restricted_point = restriction[restriction_key]
        if (point["x"] == restricted_point["x"] and
            point["y"] == restricted_point["y"] and
            point["a"] == restricted_point["a"] and
            point["s"] == restricted_point["s"]):
            return True
    return False
def all_distances_within_limit(points1, points2, max_distance):
    for point1 in points1:
        for point2 in points2:
            if distance(point1, point2) > max_distance:
                return False
    return True

def generate_random_point_around(center, half_size):
    x_min = max(rectangle["x_min"], center["x"] - half_size)
    x_max = min(rectangle["x_max"], center["x"] + half_size)
    y_min = max(rectangle["y_min"], center["y"] - half_size)
    y_max = min(rectangle["y_max"], center["y"] + half_size)

    x = random.randint(x_min, x_max)
    y = random.randint(y_min, y_max)
    heading = random.randint(0, 11)
    velocity = 2
    return {"x": x, "y": y, "a": heading, "s": velocity, "t": 0}

def generate_experiments(rectangle, output_filename, experiment_number, vehicle_count, unleaveable, unenterable, max_distance_starts, max_distance_targets, max_distance_start_target):
    experiments = []

    for i in range(experiment_number):
        print(i)
        while True:
            
            first_start = generate_random_point(rectangle)
            starts = [first_start] + [generate_random_point_around(first_start, max_distance_starts // 2) for _ in range(vehicle_count - 1)]
            
            first_target = generate_random_point(rectangle)
            targets = [first_target] + [generate_random_point_around(first_target, max_distance_targets // 2) for _ in range(vehicle_count - 1)]


            starts_valid = all([not point_in_restrictions(pos, unleaveable, "start_pose") for pos in starts])
            targets_valid = all([not point_in_restrictions(pos, unenterable, "target_pose") for pos in targets])

            starts_distances_within_limit = all_distances_within_limit(starts, starts, max_distance_starts)
            targets_distances_within_limit = all_distances_within_limit(targets, targets, max_distance_targets)

            start_target_distances_within_limit = all_distances_within_limit(starts, targets, max_distance_start_target)
            
            # print(f"starts_valid {starts_valid} targets_valid {targets_valid} starts_distances_within_limit {starts_distances_within_limit} targets_distances_within_limit {targets_distances_within_limit} start_target_distances_within_limit {start_target_distances_within_limit}")
            if starts_valid and targets_valid and starts_distances_within_limit and targets_distances_within_limit and start_target_distances_within_limit:
                break

        experiment = {
            "starts": starts,
            "targets": targets
        }
        experiments.append(experiment)

    with open(output_filename, "w") as output_file:
        json.dump(experiments, output_file, indent=4)

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Generate experiments for vehicle routing.")
parser.add_argument("input_filename", help="Input file to load the rectangle coordinates from")
parser.add_argument("output_filename", help="Output file to store generated experiments")
parser.add_argument("restrictions_directory", help="Directory containing unleaveable.json and unenterable.json files")
parser.add_argument("experiment_number", type=int, help="Number of experiments to generate")
parser.add_argument("vehicle_count", type=int, help="Number of vehicles for each experiment")
parser.add_argument("min_distance_starts", type=int, help="Grid points between start positions")
parser.add_argument("min_distance_targets", type=int, help="Grid points between target positions")
parser.add_argument("min_distance_start_target", type=int, help="Minimum grid points between a start position and its corresponding target position")

# Parse command-line arguments
args = parser.parse_args()

# Load rectangle coordinates from input file
with open(args.input_filename, "r") as input_file:
    rectangle = json.load(input_file)

# Load unleaveable and unenterable configurations from files
with open(f"{args.restrictions_directory}/unleaveable.json", "r") as unleaveable_file:
    unleaveable = json.load(unleaveable_file)
with open(f"{args.restrictions_directory}/unenterable.json", "r") as unenterable_file:
    unenterable = json.load(unenterable_file)

# Call generate_experiments function with specified arguments
generate_experiments(rectangle, args.output_filename, args.experiment_number, args.vehicle_count, unleaveable, unenterable, args.min_distance_starts, args.min_distance_targets, args.min_distance_start_target)