import os
import json
import matplotlib.pyplot as plt
import networkx as nx
import os

def load_nodes(directory):
    nodes = []
    for file in os.listdir(directory):
        if file.startswith("node") and file.endswith(".json"):
            with open(os.path.join(directory, file), 'r') as f:
                nodes.append(json.load(f))
    return nodes

def check_duplicates(lst):
    seen = []
    for item in lst:
        item_tuple = tuple(item.items())
        if item_tuple in seen:
            return True
        seen.append(item_tuple)
    return False

def visualize_nodes(nodes):
    G = nx.DiGraph()
    labels = {}
    for node in nodes:
        node_id = node["node_id"]
        sic = node["sic"]
        father_id = node["father"]
        if father_id >= 0:
            G.add_edge(father_id, node_id)
        
        if "avoid" in node:
            avoid_constraints = ", ".join([f"{constr['x']},{constr['y']},{constr['t']}" for constr in node["avoid"]])
            labels[node_id] = f"{node_id}:{sic}:{avoid_constraints}"

        if "avoid" in node:
            if check_duplicates(node["avoid"]):
                print(f"Node {node_id} contains duplicate avoid constraints.")

    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog="dot")
    plt.figure(figsize=(40, 40))
    nx.draw(G, pos, with_labels=False, arrows=True, node_size=3000, node_color="lightblue")
    nx.draw_networkx_labels(G, pos, labels, font_size=10)
    plt.title("Node Tree with Associated Avoid Constraints")
    plt.savefig("../cbs_tree.png")

nodes = load_nodes(os.getcwd())
visualize_nodes(nodes)
