"""
This script converts data dumps from dynamic graph to a RAI compatible npz format.

Author: Julian Viereck
Date: 13 Dec 2018
"""

from os import listdir
from os.path import isfile, isdir, join, basename
import matplotlib.pyplot as plt
import numpy as np
import RAI
import sys

if len(sys.argv) <= 1:
  print("Usage: python script.py <folder with the dg data folders>")
  sys.exit(0)


data_path = sys.argv[1]

dat_folders = [f for f in listdir(data_path) if isdir(join(data_path, f))]

for folder in dat_folders:
  print("Opening: ", folder)
  out_file = join(folder, folder +  ".npz")
  if True: #not isfile(out_file):

    dat_files = [join(folder,f) for f in listdir(folder) if isfile(join(folder, f)) and f.endswith('.dat')]

    for data_file in dat_files:
      print (data_file)

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
        if not data.shape == (0,):
          names = [basename(dat_file) + '[' + str(i-1) + ']' for i in range(1, data.shape[1])]
          units = ['1' for i in range(1, data.shape[1])]

          for it in range(min_timesteps):
              dc.add_vector(data[it, 1:], names, units)

    # Add special "time" array.
    print(min_timesteps)
    for it in range(min_timesteps):
        dc.add_variable(0.001 * it, 'time', 's')

    print("Writing file to:", out_file)
    RAI.data_collector.DataCollector.dump_npz(out_file, [dc])
