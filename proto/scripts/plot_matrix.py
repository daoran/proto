#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pylab as plt

import scipy
import scipy.linalg

if __name__ == "__main__":
  parser = argparse.ArgumentParser(prog='plot_matrix.py')
  parser.add_argument('--input', required=True)
  args = parser.parse_args()

  A = np.genfromtxt(args.input, delimiter=",")

  plt.matshow(A)
  plt.colorbar()
  plt.show()
