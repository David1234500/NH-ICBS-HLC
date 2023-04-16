import pulp
import json



def fit_rectangles(data, max_x, max_y, max_rectangles=3):

    complete_grid_set = []

    coords = {}
    rectangles = []
    
    is_above = []
    is_below = []
    is_right = []
    is_left = []

    problem = pulp.LpProblem("Rectangle Fitting", pulp.LpMaximize)
    for i in range(max_x + 1):
        for j in range(max_y + 1):
            complete_grid_set.append((i,j))
            for k in range(max_rectangles):
                coords[i,j,k] = pulp.LpVariable(f"x{i}_y{j}_r{k}", cat="Binary")

    for k in range(max_rectangles):
        rectangles.append( (pulp.LpVariable(f"x{k}", lowBound=0), pulp.LpVariable(f"y{k}", lowBound=0), pulp.LpVariable(f"w{k}", lowBound=0), pulp.LpVariable(f"h{k}", lowBound=0)) )

    unique_points = []
    [unique_points.append(t) for t in data if t not in unique_points]
    cut_set = []
    [cut_set.append(t) for t in complete_grid_set if t not in unique_points]
    # print(unique_points)
    # print(cut_set)
    problem += pulp.lpSum([coords[i,j,k] for (i,j) in unique_points for k in range(max_rectangles)]) - 2 * pulp.lpSum([coords[i,j,k] for (i,j) in cut_set for k in range(max_rectangles)]), "Total Inliers"

    ## Include as many as possible in rects
    for i, (x, y) in enumerate(complete_grid_set):
        for k, (rx, ry, rw, rh) in enumerate(rectangles): 
            problem += x >= rx + (1 - coords[x,y,k]) * (-1e6), f"Left_Constraint_{x}_{y}_{k}"
            problem += y >= ry + (1 - coords[x,y,k]) * (-1e6), f"Bottom_Constraint_{x}_{y}_{k}"
            problem += x <= rx + rw + coords[x,y,k] * 1e6, f"Right_Constraint_{x}_{y}_{k}"
            problem += y <= ry + rh + coords[x,y,k] * 1e6, f"Top_Constraint_{x}_{y}_{k}"

    for k in range(max_rectangles - 1):
        rx1, _, rw1, _ = rectangles[k]
        rx2, _, _, _ = rectangles[k + 1]
        problem += rx2 >= rx1 + rw1, f"Non_Overlap_Constraint_{k}"

    # Solve the problem
    problem.solve()
    print("Status:", pulp.LpStatus[problem.status])

    # Extract the results
    results = []
    for k, (rx, ry, rw, rh) in enumerate(rectangles):
        results.append({
            "x": rx.varValue,
            "y": ry.varValue,
            "w": rw.varValue,
            "h": rh.varValue
        })

    return results

# Load data from file
with open("unreachable_configs.json", "r") as file:
    configs = json.load(file)

# Process data and find grid size
data_points = {}
max_x = 0
max_y = 0
for config in configs:
    start_pose = config["start_pose"]
    x, y, a = start_pose["x"], start_pose["y"], start_pose["a"]
    
    if a not in data_points:
        data_points[a] = []
    data_points[a].append((x, y))
    
    max_x = max(max_x, x)
    max_y = max(max_y, y)

# fit_rectangles(data_points, 5)

results = {}
for a, points in data_points.items():
    results[a] = fit_rectangles(points, max_x, max_y)
    print(f"Results for start heading a={a}: {results[a]}")