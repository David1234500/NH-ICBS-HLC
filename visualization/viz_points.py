import json
import matplotlib.pyplot as plt
import math
import sys


f = open(sys.argv[1], "r")
x = f.read() 
points = json.loads(x)
  
x_values = [point['x'] for point in points]
y_values = [point['y'] for point in points]

# Plot the points using matplotlib
plt.scatter(x_values, y_values)

# Label axes
plt.xlabel('x')
plt.ylabel('y')

# Show the plot
plt.show()
