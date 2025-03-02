import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python plot_graph.py <csv_file>")
    sys.exit(1)

csv_file = sys.argv[1]

data = pd.read_csv(csv_file)
print(data.columns)

plt.figure(figsize=(8, 5))
plt.plot(data['Frame'], data['Y1'], marker='o', linestyle='-', color='r', label="Y1") 
plt.plot(data['Frame'], data['Y2'], marker='o', linestyle='-', color='b', label="Y2")

plt.xlabel("X values")
plt.ylabel("Y values")
plt.title("Graph of comparision")
plt.legend()
plt.grid(True)

png_file = os.path.splitext(csv_file)[0] + ".png"

plt.savefig(png_file)
plt.show()