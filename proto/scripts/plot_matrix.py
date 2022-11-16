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

  A_binary = np.zeros(A.shape)
  for i in range(A.shape[0]):
    for j in range(A.shape[1]):
      if abs(A[i, j]) > 1e-14:
        A_binary[i, j] = 1.0
      else:
        A_binary[i, j] = 0.0

  plt.subplot(211)
  plt.imshow(A_binary, cmap='Greys')
  plt.colorbar()

  plt.subplot(212)
  plt.imshow(abs(A))
  plt.colorbar()

  plt.show()
