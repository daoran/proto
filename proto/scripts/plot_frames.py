#!/usr/bin/env python3
import proto
import numpy as np
from numpy import deg2rad
import matplotlib.pylab as plt



plt.figure()
ax = plt.axes(projection='3d')

r_WB = np.array([0.0, 0.0, 0.0])
C_WB = proto.euler321(deg2rad(0.0), deg2rad(10.0), deg2rad(0.0))
T_WB = proto.tf(C_WB, r_WB)

r_ned_enu = np.array([0.0, 0.0, 0.0])
C_ned_enu = proto.euler321(deg2rad(0.0), deg2rad(0.0), deg2rad(180.0))
T_ned_enu = proto.tf(C_ned_enu, r_ned_enu)
proto.plot_tf(ax, T_ned_enu @ T_WB)

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
proto.plot_set_axes_equal(ax)
plt.show()
