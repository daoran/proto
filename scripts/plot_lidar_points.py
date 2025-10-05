import numpy as np
import matplotlib.pylab as plt


pcd0 = np.genfromtxt("/tmp/pcd0.csv", delimiter=" ")
pcd1 = np.genfromtxt("/tmp/pcd1.csv", delimiter=" ")

fig = plt.figure(figsize=(16, 12))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(pcd0[:, 0], pcd0[:, 1], pcd0[:, 2], c='r', marker='o', alpha=0.1)
ax.scatter(pcd1[:, 0], pcd1[:, 1], pcd1[:, 2], c='g', marker='o', alpha=0.1)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
plt.show()
