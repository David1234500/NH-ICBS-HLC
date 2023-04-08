import os
import json
import glob

def check_unique_constraints(node):
    constraints = set()
    for constraint in node['avoid']:
        key = (constraint['x'], constraint['y'], constraint['id'], constraint['t'])
        if key in constraints:
            return False
        constraints.add(key)
    return True

def check_all_nodes():
    # Get all the JSON files
    filenames = glob.glob('node*.json')

    # Load JSON data for each file and build a dictionary with node_id as key
    nodes = {}
    for filename in filenames:
        with open(filename, 'r') as f:
            node_data = json.load(f)
            nodes[node_data['node_id']] = node_data

    # Check uniqueness for each leaf node
    all_leaves_unique = True
    for node_id, node in nodes.items():
        is_leaf = True
        for other_node in nodes.values():
            if other_node['father'] == node_id:
                is_leaf = False
                break
        if is_leaf:
            unique = check_unique_constraints(node)
            if not unique:
                print(f'Node {node_id} has non-unique constraints.')
                all_leaves_unique = False

    if all_leaves_unique:
        print('All leaf nodes have unique constraints.')

if __name__ == '__main__':
    check_all_nodes()
