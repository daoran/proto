import argparse

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("infile")
  parser.add_argument("delimiter", default=" ")
  args = parser.parse_args()

  df = pd.read_csv(args.infile, delimiter=args.delimiter)
  x, y, z = df["x"], df["y"], df["z"]

  fig = plt.figure(figsize=(8, 8))
  ax = fig.add_subplot(111, projection="3d")
  ax.scatter(x, y, z, c=z, cmap="viridis", s=1)
  ax.set_xlabel("x")
  ax.set_ylabel("y")
  ax.set_zlabel("z")
  plt.show()
