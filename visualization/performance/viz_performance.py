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
    for y, (stage, calls) in enumerate(data_dict.items()):
        for call in calls.values():
            concurrent = call["concurrent_calls"] > 1
            height = 0.4 if concurrent else 0.8
            color = "orange" if concurrent else "blue"
            ax2.barh(
                y,
                call["duration"],
                left=call["start_time"],
                height=height,
                color=color,
                alpha=0.8,
                label="Concurrent" if concurrent else "Non-concurrent",
            )

    # Remove duplicate labels in the legend
    handles, labels = ax2.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax2.legend(by_label.values(), by_label.keys())

    ax2.set_yticks(y_positions)
    ax2.set_yticklabels(data_dict.keys())
    ax2.set_xlabel("Time")
    ax2.set_title("Start and End Times for Each Stage")

    plt.tight_layout()
    plt.show()
