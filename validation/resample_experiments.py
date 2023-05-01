import json
import argparse

def resample_points(input_file, output_file, input_pos_discretization, output_pos_discretization, input_head_discretization, output_head_discretization):
    with open(input_file, 'r') as f:
        experiments = json.load(f)

    resampled_experiments = []

    for experiment in experiments:
        resampled_experiment = {
            'starts': [],
            'targets': []
        }

        for start in experiment['starts']:
            resampled_start = {
                'x': round(start['x'] * input_pos_discretization / output_pos_discretization),
                'y': round(start['y'] * input_pos_discretization / output_pos_discretization),
                'a': round(start['a'] * input_head_discretization / output_head_discretization),
                's': start['s'],
                't': start['t']
            }
            resampled_experiment['starts'].append(resampled_start)

        for target in experiment['targets']:
            resampled_target = {
                'x': round(target['x'] * input_pos_discretization / output_pos_discretization),
                'y': round(target['y'] * input_pos_discretization / output_pos_discretization),
                'a': round(target['a'] * input_head_discretization / output_head_discretization),
                's': target['s'],
                't': target['t']
            }
            resampled_experiment['targets'].append(resampled_target)

        resampled_experiments.append(resampled_experiment)

    with open(output_file, 'w') as f:
        json.dump(resampled_experiments, f, indent=4)

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Resample experiments for vehicle routing.")
parser.add_argument("input_file", help="Input file containing experiments to resample")
parser.add_argument("output_file", help="Output file to store resampled experiments")
parser.add_argument("input_pos_discretization", type=float, help="Input position discretization in cm")
parser.add_argument("output_pos_discretization", type=float, help="Output position discretization in cm")
parser.add_argument("input_head_discretization", type=float, help="Input heading discretization")
parser.add_argument("output_head_discretization", type=float, help="Output heading discretization")

# Parse command-line arguments
args = parser.parse_args()

# Call resample_points function with specified arguments
resample_points(args.input_file, args.output_file, args.input_pos_discretization, args.output_pos_discretization, args.input_head_discretization, args.output_head_discretization)
