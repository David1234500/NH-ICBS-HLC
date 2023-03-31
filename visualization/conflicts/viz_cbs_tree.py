import os
import json
import matplotlib.pyplot as plt
import networkx as nx
import os
from matplotlib.patches import Rectangle

def load_nodes(directory):
    nodes = []
    for file in os.listdir(directory):
        if file.startswith("node") and file.endswith(".json"):
            with open(os.path.join(directory, file), 'r') as f:
                nodes.append(json.load(f))
    return nodes

def unique_constraints(nodes):
    constraints = set()
    for node in nodes:
        if "avoid" in node:
            for constr in node["avoid"]:
                constraints.add(tuple(constr.items()))
    return list(constraints)

def visualize_nodes(nodes, unique_constraints):
    G = nx.DiGraph()
    labels = {}
    for node in nodes:
        node_id = node["node_id"]
        sic = node["sic"]
        father_id = node["father"]
        if father_id >= 0:
            G.add_edge(father_id, node_id)
        labels[node_id] = f"{node_id}:{sic}"

    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog="dot")
    plt.figure(figsize=(120, 90))
    nx.draw(G, pos, with_labels=False, arrows=True, node_size=3000, node_color="lightblue")
    nx.draw_networkx_labels(G, pos, labels, font_size=13)

    cmap = plt.cm.get_cmap("viridis")
    constraint_colors = {c: cmap(i / len(unique_constraints)) for i, c in enumerate(unique_constraints)}

    ax = plt.gca()
    for node in nodes:
        if "avoid" in node:
            node_id = node["node_id"]
            x, y = pos[node_id]
            for idx, constr in enumerate(node["avoid"]):
                constr_id = unique_constraints.index(tuple(constr.items()))
                color = constraint_colors[tuple(constr.items())]
                rect = Rectangle((x + 10 + 5 * idx, y - 10), 5, 20, facecolor=color)
                ax.add_patch(rect)
                ax.annotate(str(constr_id), (x + 12 + 5 * idx, y - 5), color='black', fontsize=15)

    # Add legend
    legend_elements = []
    for idx, constr in enumerate(unique_constraints):
        color = constraint_colors[constr]
        label = f"{idx}: {dict(constr)['x']}, {dict(constr)['y']}, {dict(constr)['t']}"
        legend_elements.append(Rectangle((0, 0), 1, 1, facecolor=color, label=label))
    ax.legend(handles=legend_elements, title="Constraints", loc="best")

    plt.title("Node Tree with Associated Avoid Constraints")
    plt.savefig("../cbs_tree.png")

nodes = load_nodes(os.getcwd())
unique_constrs = unique_constraints(nodes)
visualize_nodes(nodes, unique_constrs)
