import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python plot_graph.py <csv_file> <technique1 optional> <technique2 optional> <joint optional> <Axis optional>")
    sys.exit(1)

csv_file = sys.argv[1]

label1 = "Y1"
label2 = "Y2"
YAxisLabel = "Angle"
Title = "Interpolation Comparision"
if len(sys.argv) >=6:
    label1 = sys.argv[2]
    label2 = sys.argv[3]
    YAxisLabel = sys.argv[4] + " - Angle " + sys.argv[5]
    Title = label1 + " vs " + label2
    

data = pd.read_csv(csv_file)

plt.figure(figsize=(8, 5))
plt.plot(data['Frame'], data['Y1'], marker='o', linestyle='-', color='r', label=label1, markersize=2) 
plt.plot(data['Frame'], data['Y2'], marker='o', linestyle='-', color='b', label=label2, markersize=2)

plt.xlabel("Frame")
plt.ylabel(YAxisLabel)
plt.title(Title)
plt.legend()
plt.grid(True)

png_file = os.path.splitext(csv_file)[0] + ".png"

plt.savefig(png_file)
plt.show()