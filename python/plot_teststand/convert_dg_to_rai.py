"""
This script converts data dumps from dynamic graph to a RAI compatible npz format.

Author: Julian Viereck
Date: 13 Dec 2018
"""

from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
import numpy as np
import RAI
import sys

if len(sys.argv) <= 1:
  print("Usage: python script.py <folder with dg dat files>")
  sys.exit(0)


data_path = sys.argv[1]
dat_files = [f for f in listdir(data_path) if isfile(join(data_path, f)) and f.endswith('.dat')]

# The datacollector to put the data into.
dc = RAI.data_collector.DataCollector()

min_timesteps = np.inf
dat_content = {}

for dat_file in dat_files:
    dat_content[dat_file] = data = np.genfromtxt(join(data_path, dat_file))

    print("Reading:", dat_file, data.shape[0])
    min_timesteps = min(min_timesteps, data.shape[0])

for dat_file in dat_files:
    data = dat_content[dat_file]
    names = [dat_file + '[' + str(i-1) + ']' for i in range(1, data.shape[1])]
    units = ['1' for i in range(1, data.shape[1])]

    for it in range(min_timesteps):
        dc.add_vector(data[it, 1:], names, units)

# Add special "time" array.
for it in range(min_timesteps):
    dc.add_variable(0.001 * it, 'time', 's')


index = len(listdir("./data"))
out_file = "D" + str(int(index) + 1) +  ".npz"
print("Writing file to:", out_file)
RAI.data_collector.DataCollector.dump_npz("./data/" + out_file, [dc])
