import json
import matplotlib.pyplot as plt
import networkx as nx
import sys

with open(sys.argv[1], 'r') as f:
    data_dict = json.loads(f.read())

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# First subplot: Average compute times for each stage
average_durations = {
    stage: sum([call["duration"] for call in calls.values()]) / len(calls)
    for stage, calls in data_dict.items()
}

ax1.bar(average_durations.keys(), average_durations.values())
ax1.set_xlabel("Stage")
ax1.set_ylabel("Average Duration")
ax1.set_title("Average Compute Times for Each Stage")

# Second subplot: Start and end times for each stage with concurrent calls
y_positions = range(len(data_dict))
first_start_time = min(min(call["start_time"] for call in calls.values()) for calls in data_dict.values())


for y, (stage, calls) in enumerate(data_dict.items()):
    concurrent_call_index = 0
    for call in calls.values():
        concurrent = call["concurrent_calls"] > 1
        height = 0.2
        
        color = "green"

        ax2.barh(
            y + 0.25 * concurrent_call_index,
            call["duration"],
            left=call["start_time"] - first_start_time,
            height=height,
            color=color,
            alpha=0.8,
        )
        if concurrent:
            concurrent_call_index += 1
        else:
            concurrent_call_index = 0

ax2.set_yticks(y_positions)
ax2.set_yticklabels(data_dict.keys())
ax2.set_xlabel("Time")
ax2.set_title("Start and End Times for Each Stage")

plt.tight_layout()
plt.savefig('../performance.png')
