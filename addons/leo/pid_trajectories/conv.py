"""RL data container."""

import os
import re
import sys
import numpy as np
import matplotlib.pyplot as plt
from collections import OrderedDict
import argparse

hd = """COLUMNS:
time[0], 
state0[0], 
state0[1], 
state0[2], 
state0[3], 
state0[4], 
state0[5], 
state0[6], 
state0[7], 
state0[8], 
state0[9], 
state0[10], 
state0[11], 
state0[12], 
state0[13], 
state0[14], 
state0[15]
DATA:"""

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    args = parser.parse_args()

    data = np.loadtxt(args.filename, skiprows=44, delimiter=',')
    ts = data[:, 0]       # time
    xs = data[:, 1:17]    # states
    nxs = np.empty(xs.shape)

    nxs[:, 0] = xs[:, 7]
    nxs[:, 1] = xs[:, 6]
    nxs[:, 2] = xs[:, 1]
    nxs[:, 3] = xs[:, 0]
    nxs[:, 4] = xs[:, 3]
    nxs[:, 5] = xs[:, 2]
    nxs[:, 6] = xs[:, 5]
    nxs[:, 7] = xs[:, 4]

    xdo = 8

    nxs[:, xdo+0] = xs[:, xdo+7]
    nxs[:, xdo+1] = xs[:, xdo+6]
    nxs[:, xdo+2] = xs[:, xdo+1]
    nxs[:, xdo+3] = xs[:, xdo+0]
    nxs[:, xdo+4] = xs[:, xdo+3]
    nxs[:, xdo+5] = xs[:, xdo+2]
    nxs[:, xdo+6] = xs[:, xdo+5]
    nxs[:, xdo+7] = xs[:, xdo+4]

    # save
    [fn, ext] = os.path.splitext( os.path.basename(args.filename))
    np.savetxt(fn+"-converted"+ext, np.column_stack((ts,nxs)), fmt='%11.6f', delimiter=',', newline='\n', header=hd, comments='')

if __name__ == "__main__":
    main()



