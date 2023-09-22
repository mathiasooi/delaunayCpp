from matplotlib import collections as mc
import matplotlib.pyplot as plt
import numpy as np
import pylab as pl
import subprocess

x = subprocess.Popen(r"g++ delaunay.cpp -std=c++17 && .\a.exe", shell=True, stdout=subprocess.PIPE).stdout.read()

edges = x.decode("ascii").strip().splitlines()


lines = np.zeros(shape=(len(edges), 2, 2))
for i, edge in enumerate(edges):
    edge = list(map(float, edge.split()))
    lines[i][0][0] = edge[0]
    lines[i][0][1] = edge[1]
    lines[i][1][0] = edge[2]
    lines[i][1][1] = edge[3]

print(lines)

lc = mc.LineCollection(lines, linewidth=2)
fig, ax = pl.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

plt.show()