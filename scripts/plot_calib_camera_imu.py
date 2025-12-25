#!/usr/bin/env python3
from pathlib import Path

import typing
from typing import cast
from typing import TypeVar
from typing import Annotated
from typing import Literal
from typing import Any
from typing import Callable

import yaml
import matplotlib.pylab as plt

import numpy as np
from numpy.typing import NDArray

DType = TypeVar("DType", bound=np.generic)
Vec2 = Annotated[NDArray[DType], Literal[2]]
Vec3 = Annotated[NDArray[DType], Literal[3]]
Vec4 = Annotated[NDArray[DType], Literal[4]]
Vec5 = Annotated[NDArray[DType], Literal[5]]
Vec6 = Annotated[NDArray[DType], Literal[6]]
Vec7 = Annotated[NDArray[DType], Literal[7]]
VecN = Annotated[NDArray[DType], Literal["N"]]
Mat2 = Annotated[NDArray[DType], Literal[2, 2]]
Mat3 = Annotated[NDArray[DType], Literal[3, 3]]
Mat34 = Annotated[NDArray[DType], Literal[3, 4]]
Mat4 = Annotated[NDArray[DType], Literal[4, 4]]
MatN = Annotated[NDArray[DType], Literal["N", "N"]]
MatNx2 = Annotated[NDArray[DType], Literal["N", "2"]]
MatNx3 = Annotated[NDArray[DType], Literal["N", "3"]]
MatNx4 = Annotated[NDArray[DType], Literal["N", "4"]]
Mat2xN = Annotated[NDArray[DType], Literal["2", "N"]]
Mat2x3 = Annotated[NDArray[DType], Literal["2", "3"]]
Mat3x4 = Annotated[NDArray[DType], Literal["3", "4"]]
Mat3xN = Annotated[NDArray[DType], Literal["3", "N"]]
Mat4xN = Annotated[NDArray[DType], Literal["4", "N"]]
Image = Annotated[NDArray[DType], Literal["N", "N"]]


def quat2rot(q: Vec4) -> Mat3:
  """
  Convert quaternion to 3x3 rotation matrix.

  Source:
  Blanco, Jose-Luis. "A tutorial on se (3) transformation parameterizations
  and on-manifold optimization." University of Malaga, Tech. Rep 3 (2010): 6.
  [Page 18, Equation (2.20)]
  """
  assert len(q) == 4
  qw, qx, qy, qz = q

  qx2 = qx**2
  qy2 = qy**2
  qz2 = qz**2
  qw2 = qw**2

  # Homogeneous form
  C11 = qw2 + qx2 - qy2 - qz2
  C12 = 2.0 * (qx * qy - qw * qz)
  C13 = 2.0 * (qx * qz + qw * qy)

  C21 = 2.0 * (qx * qy + qw * qz)
  C22 = qw2 - qx2 + qy2 - qz2
  C23 = 2.0 * (qy * qz - qw * qx)

  C31 = 2.0 * (qx * qz - qw * qy)
  C32 = 2.0 * (qy * qz + qw * qx)
  C33 = qw2 - qx2 - qy2 + qz2

  return np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])


def tf(rot: Mat3 | Vec4, trans: Vec3) -> Mat4:
  """
  Form 4x4 homogeneous transformation matrix from rotation `rot` and
  translation `trans`. Where the rotation component `rot` can be a rotation
  matrix or a quaternion.
  """
  C = None
  if rot.shape == (4,) or rot.shape == (4, 1):
    C = quat2rot(rot)
  elif rot.shape == (3, 3):
    C = rot
  else:
    raise RuntimeError("Invalid rotation!")

  T = np.eye(4, 4)
  T[0:3, 0:3] = C
  T[0:3, 3] = trans
  return T


def tf_trans(T: Mat4) -> Vec3:
  """Return translation vector from 4x4 homogeneous transform"""
  assert T.shape == (4, 4)
  return T[0:3, 3]


def tf_point(T: Mat4, p: Vec3) -> Vec3:
  """Transform 3d point"""
  assert T.shape == (4, 4)
  assert p.shape == (3,) or p.shape == (3, 1)
  hpoint = np.array([p[0], p[1], p[2], 1.0])
  return (T @ hpoint)[0:3]


def pose2tf(pose_vec: Vec7) -> Mat4:
  """Convert pose vector to transformation matrix"""
  rx, ry, rz = pose_vec[0:3]
  qx, qy, qz, qw = pose_vec[3:7]
  return tf(np.array([qw, qx, qy, qz]), np.array([rx, ry, rz]))


def plot_tf(ax, T, **kwargs):
  """
  Plot 4x4 Homogeneous Transform

  Args:

    ax (matplotlib.axes.Axes): Plot axes object
    T (np.array): 4x4 homogeneous transform (i.e. Pose in the world frame)

  Keyword args:

    size (float): Size of the coordinate-axes
    linewidth (float): Thickness of the coordinate-axes
    name (str): Frame name
    name_offset (np.array or list): Position offset for displaying the frame's name
    fontsize (float): Frame font size
    fontweight (float): Frame font weight
    colors (tuple of floats): Axes colors in x, y and z

  """
  assert T.shape == (4, 4)

  size = kwargs.get("size", 0.1)
  linewidth = kwargs.get("linewidth", 2)
  name = kwargs.get("name", None)
  nameoffset = kwargs.get("nameoffset", [0, 0, -0.01])
  fontsize = kwargs.get("fontsize", 10)
  fontweight = kwargs.get("fontweight", "bold")
  fontcolor = kwargs.get("fontcolor", "k")
  colors = kwargs.get("colors", ["r-", "g-", "b-"])

  origin = tf_trans(T)
  lx = tf_point(T, np.array([size, 0.0, 0.0]))
  ly = tf_point(T, np.array([0.0, size, 0.0]))
  lz = tf_point(T, np.array([0.0, 0.0, size]))

  # Draw x-axis
  px = [origin[0], lx[0]]
  py = [origin[1], lx[1]]
  pz = [origin[2], lx[2]]
  xaxis = ax.plot(px, py, pz, colors[0], linewidth=linewidth)[0]

  # Draw y-axis
  px = [origin[0], ly[0]]
  py = [origin[1], ly[1]]
  pz = [origin[2], ly[2]]
  yaxis = ax.plot(px, py, pz, colors[1], linewidth=linewidth)[0]

  # Draw z-axis
  px = [origin[0], lz[0]]
  py = [origin[1], lz[1]]
  pz = [origin[2], lz[2]]
  zaxis = ax.plot(px, py, pz, colors[2], linewidth=linewidth)[0]

  # Draw label
  if name is not None:
    x = origin[0] + nameoffset[0]
    y = origin[1] + nameoffset[1]
    z = origin[2] + nameoffset[2]
    text = ax.text(
      x, y, z, name, fontsize=fontsize, fontweight=fontweight, color=fontcolor
    )
    return (xaxis, yaxis, zaxis, text)

  return (xaxis, yaxis, zaxis)


def plot_set_axes_equal(ax):
  """
  Make axes of 3D plot have equal scale so that spheres appear as spheres,
  cubes as cubes, etc..  This is one possible solution to Matplotlib's
  ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

  Input
    ax: a matplotlib axis, e.g., as output from plt.gca().
  """
  x_limits = ax.get_xlim3d()
  y_limits = ax.get_ylim3d()
  z_limits = ax.get_zlim3d()

  x_range = abs(x_limits[1] - x_limits[0])
  x_middle = np.mean(x_limits)
  y_range = abs(y_limits[1] - y_limits[0])
  y_middle = np.mean(y_limits)
  z_range = abs(z_limits[1] - z_limits[0])
  z_middle = np.mean(z_limits)

  # The plot bounding box is a sphere in the sense of the infinity
  # norm, hence I call half the max range the plot radius.
  plot_radius = 0.5 * max([x_range, y_range, z_range])

  ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
  ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
  ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


data_path = Path("/tmp/calib_camera_imu.yaml")
data_file = open(data_path, "r")
data = yaml.safe_load(data_file)
data_file.close()

T_WS = pose2tf(data["sensor_pose"])
T_WF = pose2tf(data["fiducial_pose"])
T_C0S = pose2tf(data["imu_extrinsic"])
T_C0C1 = pose2tf(data["camera_extrinsic"])

plt.figure()
ax = plt.axes(projection="3d")

# plot_tf(ax, T_WF, name="T_WF")
plot_tf(ax, T_WS, name="T_WS", size=0.01)
plot_tf(ax, T_WS @ np.linalg.inv(T_C0S), name="T_WC0", size=0.01)
plot_tf(ax, T_WS @ np.linalg.inv(T_C0S) @ T_C0C1, name="T_WC1", size=0.01)

plot_set_axes_equal(ax)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
plt.show()
