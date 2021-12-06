"""
Proto
"""
import os
import sys
import glob
import math
import copy
import random
import pickle
import json
from enum import Enum
from dataclasses import dataclass
from dataclasses import field
from collections import namedtuple
from types import FunctionType
from typing import Optional
from typing import List
from typing import Dict

import cv2
import yaml
import numpy as np
import scipy
import scipy.sparse
import scipy.sparse.linalg
import pandas

import cProfile
from pstats import Stats

###############################################################################
# PROFILING
###############################################################################


def profile_start():
  """ Start profile """
  prof = cProfile.Profile()
  prof.enable()
  return prof


def profile_stop(prof, **kwargs):
  """ Stop profile """
  key = kwargs.get('key', 'cumtime')
  N = kwargs.get('N', 10)

  stats = Stats(prof)
  stats.strip_dirs()
  stats.sort_stats(key).print_stats(N)


###############################################################################
# MATHS
###############################################################################

from math import pi
from math import isclose
from math import sqrt
from math import floor
from math import cos
from math import sin
from math import tan
from math import acos
from math import atan


def rmse(errors):
  """ Root Mean Squared Error """
  return np.sqrt(np.mean(errors**2))


###############################################################################
# TIME
###############################################################################


def sec2ts(time_s):
  """ Convert time in seconds to timestamp """
  return int(time_s * 1e9)


def ts2sec(ts):
  """ Convert timestamp to seconds """
  return float(ts * 1e-9)


###############################################################################
# LINEAR ALGEBRA
###############################################################################

# from numpy import rad2deg
from numpy import deg2rad
from numpy import sinc
from numpy import zeros
from numpy import ones
from numpy import eye
from numpy import diagonal as diag
from numpy import cross
from numpy.linalg import norm
from numpy.linalg import inv
from numpy.linalg import pinv
from numpy.linalg import matrix_rank as rank
from numpy.linalg import eig
from numpy.linalg import svd
from numpy.linalg import cholesky as chol


def normalize(v):
  """ Normalize vector v """
  n = np.linalg.norm(v)
  if n == 0:
    return v
  return v / n


def full_rank(A):
  """ Check if matrix A is full rank """
  return rank(A) == A.shape[0]


def skew(vec):
  """ Form skew-symmetric matrix from vector `vec` """
  assert vec.shape == (3,) or vec.shape == (3, 1)
  x, y, z = vec
  return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def skew_inv(A):
  """ Form skew symmetric matrix vector """
  assert A.shape == (3, 3)
  return np.array([A[2, 1], A[0, 2], A[1, 0]])


def fwdsubs(L, b):
  """
  Solving a lower triangular system by forward-substitution
  Input matrix L is an n by n lower triangular matrix
  Input vector b is n by 1
  Output vector x is the solution to the linear system
  L x = b
  """
  assert L.shape[1] == b.shape[0]
  n = b.shape[0]

  x = zeros((n, 1))
  for j in range(n):
    if L[j, j] == 0:
      raise RuntimeError('Matrix is singular!')
    x[j] = b[j] / L[j, j]
    b[j:n] = b[j:n] - L[j:n, j] * x[j]


def bwdsubs(U, b):
  """
  Solving an upper triangular system by back-substitution
  Input matrix U is an n by n upper triangular matrix
  Input vector b is n by 1
  Output vector x is the solution to the linear system
  U x = b
  """
  assert U.shape[1] == b.shape[0]
  n = b.shape[0]

  x = zeros((n, 1))
  for j in range(n):
    if U[j, j] == 0:
      raise RuntimeError('Matrix is singular!')
    x[j] = b[j] / U(j, j)
    b[0:j] = b[0:j] - U[0:j, j] * x[j]


def schurs_complement(H, g, m, r, precond=False):
  """ Shurs-complement """
  assert H.shape[0] == (m + r)

  # H = [Hmm, Hmr
  #      Hrm, Hrr];
  Hmm = H[0:m, 0:m]
  Hmr = H[0:m, m:]
  Hrm = Hmr.T
  Hrr = H[m:, m:]

  # g = [gmm, grr]
  gmm = g[1:]
  grr = g[m:]

  # Precondition Hmm
  if precond:
    Hmm = 0.5 * (Hmm + Hmm.T)

  # Invert Hmm
  assert rank(Hmm) == Hmm.shape[0]
  (w, V) = eig(Hmm)
  W_inv = diag(1.0 / w)
  Hmm_inv = V * W_inv * V.T

  # Schurs complement
  H_marg = Hrr - Hrm * Hmm_inv * Hmr
  g_marg = grr - Hrm * Hmm_inv * gmm

  return (H_marg, g_marg)


def matrix_equal(A, B, tol=1e-8, verbose=False):
  """ Compare matrices `A` and `B` """
  diff = A - B

  if len(diff.shape) == 1:
    for i in range(diff.shape[0]):
      if abs(diff[i]) > tol:
        if verbose:
          print("A - B:")
          print(diff)

  elif len(diff.shape) == 2:
    for i in range(diff.shape[0]):
      for j in range(diff.shape[1]):
        if abs(diff[i, j]) > tol:
          if verbose:
            print("A - B:")
            print(diff)
          return False

  return True


def check_jacobian(jac_name, fdiff, jac, threshold, verbose=True):
  """ Check jacobians """

  # Check if numerical diff is same as analytical jacobian
  if matrix_equal(fdiff, jac, threshold):
    if verbose:
      print(f"Check [{jac_name}] passed!")
    return True

  # Failed - print differences
  if verbose:
    fdiff_minus_jac = fdiff - jac

    print(f"Check [{jac_name}] failed!")
    print("-" * 60)

    print("J_fdiff - J:")
    print(np.round(fdiff_minus_jac, 4))
    print()

    print("J_fdiff:")
    print(np.round(fdiff, 4))
    print()

    print("J:")
    print(np.round(jac, 4))
    print()

    print("-" * 60)

  return True


###############################################################################
# GEOMETRY
###############################################################################


def circle_loss(c, x, y):
  """
    Calculate the algebraic distance between the data points and the mean
    circle centered at c=(xc, yc)
    """
  xc, yc = c
  # Euclidean dist from center (xc, yc)
  Ri = np.sqrt((x - xc)**2 + (y - yc)**2)
  return Ri - Ri.mean()


def find_circle(x, y):
  """
    Find the circle center and radius given (x, y) data points using least
    squares. Returns `(circle_center, circle_radius, residual)`
    """
  x_m = np.mean(x)
  y_m = np.mean(y)
  center_init = x_m, y_m
  center, _ = scipy.optimize.leastsq(circle_loss, center_init, args=(x, y))

  xc, yc = center
  radii = np.sqrt((x - xc)**2 + (y - yc)**2)
  radius = radii.mean()
  residual = np.sum((radii - radius)**2)

  return (center, radius, residual)


def bresenham(p0, p1):
  """
    Bresenham's line algorithm is a line drawing algorithm that determines the
    points of an n-dimensional raster that should be selected in order to form
    a close approximation to a straight line between two points. It is commonly
    used to draw line primitives in a bitmap image (e.g. on a computer screen),
    as it uses only integer addition, subtraction and bit shifting, all of
    which are very cheap operations in standard computer architectures.

    Args:

      p0 (np.array): Starting point (x, y)
      p1 (np.array): End point (x, y)

    Returns:

      A list of (x, y) intermediate points from p0 to p1.

    """
  x0, y0 = p0
  x1, y1 = p1
  dx = abs(x1 - x0)
  dy = abs(y1 - y0)
  sx = 1.0 if x0 < x1 else -1.0
  sy = 1.0 if y0 < y1 else -1.0
  err = dx - dy

  line = []
  while True:
    line.append([x0, y0])
    if x0 == x1 and y0 == y1:
      return line

    e2 = 2 * err
    if e2 > -dy:
      # overshot in the y direction
      err = err - dy
      x0 = x0 + sx
    if e2 < dx:
      # overshot in the x direction
      err = err + dx
      y0 = y0 + sy


###############################################################################
# LIE
###############################################################################


def Exp(phi):
  """ Exponential Map """
  assert phi.shape == (3,) or phi.shape == (3, 1)
  if norm(phi) < 1e-3:
    C = eye(3) + skew(phi)
    return C

  phi_norm = norm(phi)
  phi_skew = skew(phi)
  phi_skew_sq = phi_skew * phi_skew

  C = eye(3)
  C += (sin(phi_norm) / phi_norm) * phi_skew
  C += ((1 - cos(phi_norm)) / phi_norm**2) * phi_skew_sq
  return C


def Log(C):
  """ Logarithmic Map """
  assert C.shape == (3, 3)
  # phi = acos((trace(C) - 1) / 2);
  # u = skew_inv(C - C') / (2 * sin(phi));
  # rvec = phi * u;

  C00, C01, C02 = C[0, :]
  C10, C11, C12 = C[1, :]
  C20, C21, C22 = C[2, :]

  tr = np.trace(C)
  rvec = None
  if tr + 1.0 < 1e-10:
    if abs(C22 + 1.0) > 1.0e-5:
      x = np.array([C02, C12, 1.0 + C22])
      rvec = (pi / np.sqrt(2.0 + 2.0 * C22)) @ x
    elif abs(C11 + 1.0) > 1.0e-5:
      x = np.array([C01, 1.0 + C11, C21])
      rvec = (pi / np.sqrt(2.0 + 2.0 * C11)) @ x
    else:
      x = np.array([1.0 + C00, C10, C20])
      rvec = (pi / np.sqrt(2.0 + 2.0 * C00)) @ x

  else:
    tr_3 = tr - 3.0  # always negative
    if tr_3 < -1e-7:
      theta = acos((tr - 1.0) / 2.0)
      magnitude = theta / (2.0 * sin(theta))
    else:
      # when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      # use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      # see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0
    rvec = magnitude * np.array([C21 - C12, C02 - C20, C10 - C01])

  return rvec


def Jr(theta):
  """
  Right jacobian

  Forster, Christian, et al. "IMU preintegration on manifold for efficient
  visual-inertial maximum-a-posteriori estimation." Georgia Institute of
  Technology, 2015.
  [Page 2, Equation (8)]
  """
  theta_norm = norm(theta)
  theta_norm_sq = theta_norm * theta_norm
  theta_norm_cube = theta_norm_sq * theta_norm
  theta_skew = skew(theta)
  theta_skew_sq = theta_skew * theta_skew

  J = eye(3)
  J -= ((1 - cos(theta_norm)) / theta_norm_sq) * theta_skew
  J += (theta_norm - sin(theta_norm)) / (theta_norm_cube) * theta_skew_sq
  return J


def Jr_inv(theta):
  """ Inverse right jacobian """
  theta_norm = norm(theta)
  theta_norm_sq = theta_norm * theta_norm
  theta_skew = skew(theta)
  theta_skew_sq = theta_skew * theta_skew

  A = 1.0 / theta_norm_sq
  B = (1 + cos(theta_norm)) / (2 * theta_norm * sin(theta_norm))

  J = eye(3)
  J += 0.5 * theta_skew
  J += (A - B) * theta_skew_sq
  return J


def boxplus(C, alpha):
  """ Box plus """
  # C_updated = C [+] alpha
  C_updated = C * Exp(alpha)
  return C_updated


def boxminus(C_a, C_b):
  """ Box minus """
  # alpha = C_a [-] C_b
  alpha = Log(inv(C_b) * C_a)
  return alpha


###############################################################################
# TRANSFORM
###############################################################################


def homogeneous(p):
  """ Turn point `p` into its homogeneous form """
  return np.array([*p, 1.0])


def dehomogeneous(hp):
  """ De-homogenize point `hp` into `p` """
  return hp[0:3]


def rotx(theta):
  """ Form rotation matrix around x axis """
  row0 = [1.0, 0.0, 0.0]
  row1 = [0.0, cos(theta), -sin(theta)]
  row2 = [0.0, sin(theta), cos(theta)]
  return np.array([row0, row1, row2])


def roty(theta):
  """ Form rotation matrix around y axis """
  row0 = [cos(theta), 0.0, sin(theta)]
  row1 = [0.0, 1.0, 0.0]
  row2 = [-sin(theta), 0.0, cos(theta)]
  return np.array([row0, row1, row2])


def rotz(theta):
  """ Form rotation matrix around z axis """
  row0 = [cos(theta), -sin(theta), 0.0]
  row1 = [sin(theta), cos(theta), 0.0]
  row2 = [0.0, 0.0, 1.0]
  return np.array([row0, row1, row2])


def aa2quat(angle, axis):
  """
  Convert angle-axis to quaternion

  Source:
  Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv
  preprint arXiv:1711.02508 (2017).
  [Page 22, eq (101), "Quaternion and rotation vector"]
  """
  ax, ay, az = axis
  qw = cos(angle / 2.0)
  qx = ax * sin(angle / 2.0)
  qy = ay * sin(angle / 2.0)
  qz = az * sin(angle / 2.0)
  return np.array([qw, qx, qy, qz])


def rvec2rot(rvec):
  """ Rotation vector to rotation matrix """
  # If small rotation
  theta = sqrt(rvec @ rvec)  # = norm(rvec), but faster
  eps = 1e-8
  if theta < eps:
    return skew(rvec)

  # Convert rvec to rotation matrix
  rvec = rvec / theta
  x, y, z = rvec

  c = cos(theta)
  s = sin(theta)
  C = 1 - c

  xs = x * s
  ys = y * s
  zs = z * s

  xC = x * C
  yC = y * C
  zC = z * C

  xyC = x * yC
  yzC = y * zC
  zxC = z * xC

  row0 = [x * xC + c, xyC - zs, zxC + ys]
  row1 = [xyC + zs, y * yC + c, yzC - xs]
  row2 = [zxC - ys, yzC + xs, z * zC + c]
  return np.array([row0, row1, row2])


def vecs2axisangle(u, v):
  """ From 2 vectors form an axis-angle vector """
  angle = math.acos(u.T * v)
  ax = normalize(np.cross(u, v))
  return ax * angle


def euler321(yaw, pitch, roll):
  """
  Convert yaw, pitch, roll in radians to a 3x3 rotation matrix.

  Source:
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  [Page 85-86, "The Aerospace Sequence"]
  """
  psi = yaw
  theta = pitch
  phi = roll

  cpsi = cos(psi)
  spsi = sin(psi)
  ctheta = cos(theta)
  stheta = sin(theta)
  cphi = cos(phi)
  sphi = sin(phi)

  C11 = cpsi * ctheta
  C21 = spsi * ctheta
  C31 = -stheta

  C12 = cpsi * stheta * sphi - spsi * cphi
  C22 = spsi * stheta * sphi + cpsi * cphi
  C32 = ctheta * sphi

  C13 = cpsi * stheta * cphi + spsi * sphi
  C23 = spsi * stheta * cphi - cpsi * sphi
  C33 = ctheta * cphi

  return np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])


def euler2quat(yaw, pitch, roll):
  """
  Convert yaw, pitch, roll in radians to a quaternion.

  Source:
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  [Page 166-167, "Euler Angles to Quaternion"]
  """
  psi = yaw  # Yaw
  theta = pitch  # Pitch
  phi = roll  # Roll

  c_phi = cos(phi / 2.0)
  c_theta = cos(theta / 2.0)
  c_psi = cos(psi / 2.0)
  s_phi = sin(phi / 2.0)
  s_theta = sin(theta / 2.0)
  s_psi = sin(psi / 2.0)

  qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi
  qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi
  qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi
  qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi

  mag = sqrt(qw**2 + qx**2 + qy**2 + qz**2)
  return np.array([qw / mag, qx / mag, qy / mag, qz / mag])


def quat2euler(q):
  """
  Convert quaternion to euler angles (yaw, pitch, roll).

  Source:
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  [Page 168, "Quaternion to Euler Angles"]
  """
  qw, qx, qy, qz = q

  m11 = (2 * qw**2) + (2 * qx**2) - 1
  m12 = 2 * (qx * qy + qw * qz)
  m13 = 2 * qx * qz - 2 * qw * qy
  m23 = 2 * qy * qz + 2 * qw * qx
  m33 = (2 * qw**2) + (2 * qz**2) - 1

  psi = math.atan2(m12, m11)
  theta = math.asin(-m13)
  phi = math.atan2(m23, m33)

  ypr = np.array([psi, theta, phi])
  return ypr


def quat2rot(q):
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


def rot2euler(C):
  """
  Convert 3x3 rotation matrix to euler angles (yaw, pitch, roll).
  """
  assert C.shape == (3, 3)
  q = rot2quat(C)
  return quat2euler(q)


def rot2quat(C):
  """
  Convert 3x3 rotation matrix to quaternion.
  """
  assert C.shape == (3, 3)

  m00 = C[0, 0]
  m01 = C[0, 1]
  m02 = C[0, 2]

  m10 = C[1, 0]
  m11 = C[1, 1]
  m12 = C[1, 2]

  m20 = C[2, 0]
  m21 = C[2, 1]
  m22 = C[2, 2]

  tr = m00 + m11 + m22

  if tr > 0:
    S = sqrt(tr + 1.0) * 2.0
    # S=4*qw
    qw = 0.25 * S
    qx = (m21 - m12) / S
    qy = (m02 - m20) / S
    qz = (m10 - m01) / S
  elif ((m00 > m11) and (m00 > m22)):
    S = sqrt(1.0 + m00 - m11 - m22) * 2.0
    # S=4*qx
    qw = (m21 - m12) / S
    qx = 0.25 * S
    qy = (m01 + m10) / S
    qz = (m02 + m20) / S
  elif m11 > m22:
    S = sqrt(1.0 + m11 - m00 - m22) * 2.0
    # S=4*qy
    qw = (m02 - m20) / S
    qx = (m01 + m10) / S
    qy = 0.25 * S
    qz = (m12 + m21) / S
  else:
    S = sqrt(1.0 + m22 - m00 - m11) * 2.0
    # S=4*qz
    qw = (m10 - m01) / S
    qx = (m02 + m20) / S
    qy = (m12 + m21) / S
    qz = 0.25 * S

  return quat_normalize(np.array([qw, qx, qy, qz]))


# QUATERNION ##################################################################


def quat_norm(q):
  """ Returns norm of a quaternion """
  qw, qx, qy, qz = q
  return sqrt(qw**2 + qx**2 + qy**2 + qz**2)


def quat_normalize(q):
  """ Normalize quaternion """
  n = quat_norm(q)
  qw, qx, qy, qz = q
  return np.array([qw / n, qx / n, qy / n, qz / n])


def quat_conj(q):
  """ Return conjugate quaternion """
  qw, qx, qy, qz = q
  q_conj = np.array([qw, -qx, -qy, -qz])
  return q_conj


def quat_inv(q):
  """ Invert quaternion """
  return quat_conj(q)


def quat_left(q):
  """ Quaternion left product matrix """
  qw, qx, qy, qz = q
  row0 = [qw, -qx, -qy, -qz]
  row1 = [qx, qw, -qz, qy]
  row2 = [qy, qz, qw, -qx]
  row3 = [qz, -qy, qx, qw]
  return np.array([row0, row1, row2, row3])


def quat_right(q):
  """ Quaternion right product matrix """
  qw, qx, qy, qz = q
  row0 = [qw, -qx, -qy, -qz]
  row1 = [qx, qw, qz, -qy]
  row2 = [qy, -qz, qw, qx]
  row3 = [qz, qy, -qx, qw]
  return np.array([row0, row1, row2, row3])


def quat_lmul(p, q):
  """ Quaternion left multiply """
  assert len(p) == 4
  assert len(q) == 4
  lprod = quat_left(p)
  return lprod @ q


def quat_rmul(p, q):
  """ Quaternion right multiply """
  assert len(p) == 4
  assert len(q) == 4
  rprod = quat_right(q)
  return rprod @ p


def quat_mul(p, q):
  """ Quaternion multiply p * q """
  return quat_lmul(p, q)


def quat_omega(w):
  """ Quaternion omega matrix """
  return np.block([[-1.0 * skew(w), w], [w.T, 0.0]])


def quat_delta(dalpha):
  """ Form quaternion from small angle rotation vector dalpha """
  half_norm = 0.5 * norm(dalpha)
  scalar = cos(half_norm)
  vector = sinc(half_norm) * 0.5 * dalpha

  dqw = scalar
  dqx, dqy, dqz = vector
  dq = np.array([dqw, dqx, dqy, dqz])

  return dq


def quat_integrate(q_k, w, dt):
  """
  Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv
  preprint arXiv:1711.02508 (2017).
  [Section 4.6.1 Zeroth-order integration, p.47]
  """
  w_norm = norm(w)
  q_scalar = 0.0
  q_vec = np.array([0.0, 0.0, 0.0])

  if w_norm > 1e-5:
    q_scalar = cos(w_norm * dt * 0.5)
    q_vec = w / w_norm * sin(w_norm * dt * 0.5)
  else:
    q_scalar = 1.0
    q_vec = [0.0, 0.0, 0.0]

  q_kp1 = quat_mul(q_k, np.array([q_scalar, q_vec]))
  return q_kp1


# TF ##########################################################################


def tf(rot, trans):
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


def tf_rot(T):
  """ Return rotation matrix from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return T[0:3, 0:3]


def tf_quat(T):
  """ Return quaternion from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return rot2quat(tf_rot(T))


def tf_trans(T):
  """ Return translation vector from 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return T[0:3, 3]


def tf_inv(T):
  """ Invert 4x4 homogeneous transform """
  assert T.shape == (4, 4)
  return np.linalg.inv(T)


def tf_point(T, p):
  """ Transform 3d point """
  assert T.shape == (4, 4)
  assert p.shape == (3,) or p.shape == (3, 1)
  hpoint = np.array([p[0], p[1], p[2], 1.0])
  return (T @ hpoint)[0:3]


def tf_hpoint(T, hp):
  """ Transform 3d point """
  assert T.shape == (4, 4)
  assert hp.shape == (4,) or hp.shape == (4, 1)
  return (T @ hp)[0:3]


def tf_decompose(T):
  """ Decompose into rotation matrix and translation vector"""
  assert T.shape == (4, 4)
  C = tf_rot(T)
  r = tf_trans(T)
  return (C, r)


def tf_perturb(T, i, step_size):
  """ Perturb transformation matrix """
  assert T.shape == (4, 4)
  assert i >= 0 and i <= 5

  # Setup
  C = tf_rot(T)
  r = tf_trans(T)

  if i >= 0 and i <= 2:
    # Perturb translation
    r[i] += step_size

  elif i >= 3 and i <= 5:
    # Perturb rotation
    rvec = np.array([0.0, 0.0, 0.0])
    rvec[i - 3] = step_size

    q = rot2quat(C)
    dq = quat_delta(rvec)

    q_diff = quat_mul(q, dq)
    q_diff = quat_normalize(q_diff)

    C = quat2rot(q_diff)

  return tf(C, r)


def tf_update(T, dx):
  """ Update transformation matrix """
  assert T.shape == (4, 4)

  q = tf_quat(T)
  r = tf_trans(T)

  dr = dx[0:3]
  dalpha = dx[3:6]
  dq = quat_delta(dalpha)

  return tf(quat_mul(q, dq), r + dr)


###############################################################################
# YAML
###############################################################################


def load_yaml(yaml_path):
  """ Load YAML and return a named tuple """
  assert yaml_path is not None
  assert yaml_path != ""

  # Load yaml_file
  yaml_data = None
  with open(yaml_path, "r") as stream:
    yaml_data = yaml.safe_load(stream)

  # Convert dict to named tuple
  data = json.dumps(yaml_data)  # Python dict to json
  data = json.loads(
      data, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))

  return data


###############################################################################
# MATPLOTLIB
###############################################################################

import matplotlib.pylab as plt


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

  """
  assert T.shape == (4, 4)

  size = kwargs.get('size', 1)
  # linewidth = kwargs.get('linewidth', 3)
  name = kwargs.get('name', None)
  name_offset = kwargs.get('name_offset', [0, 0, -0.01])
  fontsize = kwargs.get('fontsize', 10)
  fontweight = kwargs.get('fontweight', 'bold')
  colors = kwargs.get('colors', ['r-', 'g-', 'b-'])

  r = tf_trans(T)
  origin = r
  x_axis = T @ np.array([size * 1.0, 0.0, 0.0, 1.0])
  y_axis = T @ np.array([0.0, size * 1.0, 0.0, 1.0])
  z_axis = T @ np.array([0.0, 0.0, size * 1.0, 1.0])

  # Draw x-axis
  ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]],
          [origin[2], x_axis[2]], colors[0])

  # Draw y-axis
  ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]],
          [origin[2], y_axis[2]], colors[1])

  # Draw z-axis
  ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]],
          [origin[2], z_axis[2]], colors[2])

  # Draw label
  if name is not None:
    x = origin + name_offset[0]
    y = origin + name_offset[1]
    z = origin + name_offset[2]
    ax.text(x, y, z, name, fontsize=fontsize, fontweight=fontweight)


###############################################################################
# CV
###############################################################################

# UTILS #######################################################################


def lookat(cam_pos, target_pos, **kwargs):
  """ Form look at matrix """
  up_axis = kwargs.get('up_axis', [0.0, -1.0, 0.0])
  assert cam_pos.shape == (3,) or cam_pos.shape == (3, 1)
  assert target_pos.shape == (3,) or target_pos.shape == (3, 1)
  assert up_axis.shape == (3,) or up_axis.shape == (3, 1)

  # Note: If we were using OpenGL the cam_dir would be the opposite direction,
  # since in OpenGL the camera forward is -z. In robotics however our camera is
  # +z forward.
  cam_dir = normalize((target_pos - cam_pos))
  cam_right = normalize(cross(up_axis, cam_dir))
  cam_up = cross(cam_dir, cam_right)

  A = zeros((4, 4))
  A[0, :] = [cam_right[0], cam_right[1], cam_right[2], 0.0]
  A[1, :] = [cam_up[0], cam_up[1], cam_up[2], 0.0]
  A[2, :] = [cam_dir[0], cam_dir[1], cam_dir[2], 0.0]
  A[3, :] = [0.0, 0.0, 0.0, 1.0]

  B = zeros(4, 4)
  B[0, :] = [1.0, 0.0, 0.0, -cam_pos[0]]
  B[1, :] = [0.0, 1.0, 0.0, -cam_pos[1]]
  B[2, :] = [0.0, 0.0, 1.0, -cam_pos[2]]
  B[3, :] = [0.0, 0.0, 0.0, 1.0]

  T_camera_target = A @ B
  T_target_camera = inv(T_camera_target)
  return T_target_camera


# GEOMETRY ####################################################################


def linear_triangulation(P_i, P_j, z_i, z_j):
  """
  Linear triangulation

  This function is used to triangulate a single 3D point observed by two
  camera frames (be it in time with the same camera, or two different cameras
  with known extrinsics).

  Args:

    P_i (np.array): First camera 3x4 projection matrix
    P_j (np.array): Second camera 3x4 projection matrix
    z_i (np.array): First keypoint measurement
    z_j (np.array): Second keypoint measurement

  Returns:

    p_Ci (np.array): 3D point w.r.t first camera

  """

  # First three rows of P_i and P_j
  P1T_i = P_i[0, :]
  P2T_i = P_i[1, :]
  P3T_i = P_i[2, :]
  P1T_j = P_j[0, :]
  P2T_j = P_j[1, :]
  P3T_j = P_j[2, :]

  # Image point from the first and second frame
  x_i, y_i = z_i
  x_j, y_j = z_j

  # Form the A matrix of AX = 0
  A = zeros((4, 4))
  A[0, :] = x_i * P3T_i - P1T_i
  A[1, :] = y_i * P3T_i - P2T_i
  A[2, :] = x_j * P3T_j - P1T_j
  A[3, :] = y_j * P3T_j - P2T_j

  # Use SVD to solve AX = 0
  (_, _, Vh) = svd(A.T @ A)
  hp = Vh.T[:, -1]  # Get the best result from SVD (last column of V)
  hp = hp / hp[-1]  # Normalize the homogeneous 3D point
  p = hp[0:3]  # Return only the first three components (x, y, z)
  return p


# PINHOLE #####################################################################


def focal_length(image_width, fov_deg):
  """
  Estimated focal length based on `image_width` and field of fiew `fov_deg`
  in degrees.
  """
  return (image_width / 2.0) / tan(deg2rad(fov_deg / 2.0))


def pinhole_K(params):
  """ Form camera matrix K """
  fx, fy, cx, cy = params
  return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def pinhole_P(params, T_WC):
  """ Form 3x4 projection matrix P """
  K = pinhole_K(params)
  T_CW = inv(T_WC)
  C = tf_rot(T_CW)
  r = tf_trans(T_CW)

  P = zeros((3, 4))
  P[0:3, 0:3] = C
  P[0:3, 3] = r
  P = K @ P
  return P


def pinhole_project(proj_params, p_C):
  """ Project 3D point onto image plane using pinhole camera model """
  assert len(proj_params) == 4
  assert len(p_C) == 3

  # Project
  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])

  # Scale and center
  fx, fy, cx, cy = proj_params
  z = np.array([fx * x[0] + cx, fy * x[1] + cy])

  return z


def pinhole_params_jacobian(x):
  """ Form pinhole parameter jacobian """
  return np.array([[x[0], 0.0, 1.0, 0.0], [0.0, x[1], 0.0, 1.0]])


def pinhole_point_jacobian(proj_params):
  """ Form pinhole point jacobian """
  fx, fy, _, _ = proj_params
  return np.array([[fx, 0.0], [0.0, fy]])


# RADTAN4 #####################################################################


def radtan4_distort(dist_params, p):
  """ Distort point with Radial-Tangential distortion """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, p1, p2 = dist_params

  # Point
  x, y = p

  # Apply radial distortion
  x2 = x * x
  y2 = y * y
  r2 = x2 + y2
  r4 = r2 * r2
  radial_factor = 1.0 + (k1 * r2) + (k2 * r4)
  x_dash = x * radial_factor
  y_dash = y * radial_factor

  # Apply tangential distortion
  xy = x * y
  x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2))
  y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy)
  return np.array([x_ddash, y_ddash])


def radtan4_point_jacobian(dist_params, p):
  """ Radial-tangential point jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, p1, p2 = dist_params

  # Point
  x, y = p

  # Apply radial distortion
  x2 = x * x
  y2 = y * y
  r2 = x2 + y2
  r4 = r2 * r2

  # Point Jacobian
  # Let u = [x; y] normalized point
  # Let u' be the distorted u
  # The jacobian of u' w.r.t. u (or du'/du) is:
  J_point = zeros((2, 2))
  J_point[0, 0] = k1 * r2 + k2 * r4 + 2.0 * p1 * y + 6.0 * p2 * x
  J_point[0, 0] += x * (2.0 * k1 * x + 4.0 * k2 * x * r2) + 1.0
  J_point[1, 0] = 2.0 * p1 * x + 2.0 * p2 * y
  J_point[1, 0] += y * (2.0 * k1 * x + 4.0 * k2 * x * r2)
  J_point[0, 1] = J_point[1, 0]
  J_point[1, 1] = k1 * r2 + k2 * r4 + 6.0 * p1 * y + 2.0 * p2 * x
  J_point[1, 1] += y * (2.0 * k1 * y + 4.0 * k2 * y * r2) + 1.0
  # Above is generated using sympy

  return J_point


def radtan4_undistort(dist_params, p0):
  """ Un-distort point with Radial-Tangential distortion """
  assert len(dist_params) == 4
  assert len(p0) == 2

  # Undistort
  p = p0
  max_iter = 5

  for _ in range(max_iter):
    # Error
    p_distorted = radtan4_distort(dist_params, p)
    J = radtan4_point_jacobian(dist_params, p)
    err = (p0 - p_distorted)

    # Update
    # dp = inv(J' * J) * J' * err
    dp = pinv(J) @ err
    p = p + dp

    # Check threshold
    if (err.T @ err) < 1e-15:
      break

  return p


def radtan4_params_jacobian(dist_params, p):
  """ Radial-Tangential distortion parameter jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Point
  x, y = p

  # Setup
  x2 = x * x
  y2 = y * y
  xy = x * y
  r2 = x2 + y2
  r4 = r2 * r2

  # Params Jacobian
  J_params = zeros((2, 4))
  J_params[0, 0] = x * r2
  J_params[0, 1] = x * r4
  J_params[0, 2] = 2.0 * xy
  J_params[0, 3] = 3.0 * x2 + y2
  J_params[1, 0] = y * r2
  J_params[1, 1] = y * r4
  J_params[1, 2] = x2 + 3.0 * y2
  J_params[1, 3] = 2.0 * xy

  return J_params


# EQUI4 #######################################################################


def equi4_distort(dist_params, p):
  """ Distort point with Equi-distant distortion """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  # Distort
  x, y = p
  r = sqrt(x * x + y * y)
  th = math.atan(r)
  th2 = th * th
  th4 = th2 * th2
  th6 = th4 * th2
  th8 = th4 * th4
  thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8)
  s = thd / r
  x_dash = s * x
  y_dash = s * y
  return np.array([x_dash, y_dash])


def equi4_undistort(dist_params, p):
  """ Undistort point using Equi-distant distortion """
  thd = sqrt(p(0) * p(0) + p[0] * p[0])

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  th = thd  # Initial guess
  for _ in range(20):
    th2 = th * th
    th4 = th2 * th2
    th6 = th4 * th2
    th8 = th4 * th4
    th = thd / (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8)

  scaling = tan(th) / thd
  return np.array([p[0] * scaling, p[1] * scaling])


def equi4_params_jacobian(dist_params, p):
  """ Equi-distant distortion params jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Jacobian
  x, y = p
  r = sqrt(x**2 + y**2)
  th = atan(r)

  J_params = zeros((2, 4))
  J_params[0, 0] = x * th**3 / r
  J_params[0, 1] = x * th**5 / r
  J_params[0, 2] = x * th**7 / r
  J_params[0, 3] = x * th**9 / r

  J_params[1, 0] = y * th**3 / r
  J_params[1, 1] = y * th**5 / r
  J_params[1, 2] = y * th**7 / r
  J_params[1, 3] = y * th**9 / r

  return J_params


def equi4_point_jacobian(dist_params, p):
  """ Equi-distant distortion point jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  # Jacobian
  x, y = p
  r = sqrt(x**2 + y**2)

  th = math.atan(r)
  th2 = th**2
  th4 = th**4
  th6 = th**6
  th8 = th**8
  thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8)

  th_r = 1.0 / (r * r + 1.0)
  thd_th = 1.0 + 3.0 * k1 * th2
  thd_th += 5.0 * k2 * th4
  thd_th += 7.0 * k3 * th6
  thd_th += 9.0 * k4 * th8
  s = thd / r
  s_r = thd_th * th_r / r - thd / (r * r)
  r_x = 1.0 / r * x
  r_y = 1.0 / r * y

  J_point = zeros((2, 2))
  J_point[0, 0] = s + x * s_r * r_x
  J_point[0, 1] = x * s_r * r_y
  J_point[1, 0] = y * s_r * r_x
  J_point[1, 1] = s + y * s_r * r_y

  return J_point


# PINHOLE RADTAN4 #############################################################


def pinhole_radtan4_project(proj_params, dist_params, p_C):
  """ Pinhole + Radial-Tangential project """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  # Project
  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])

  # Distort
  x_dist = radtan4_distort(dist_params, x)

  # Scale and center to image plane
  fx, fy, cx, cy = proj_params
  z = np.array([fx * x_dist[0] + cx, fy * x_dist[1] + cy])
  return z


def pinhole_radtan4_backproject(proj_params, dist_params, z):
  """ Pinhole + Radial-Tangential back-project """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(z) == 2

  # Convert image pixel coordinates to normalized retinal coordintes
  fx, fy, cx, cy = proj_params
  x = np.array([(z[0] - cx) / fx, (z[1] - cy) / fy, 1.0])

  # Undistort
  x = radtan4_undistort(dist_params, x)

  # 3D ray
  p = np.array([x[0], x[1], 1.0])
  return p


def pinhole_radtan4_undistort(proj_params, dist_params, z):
  """ Pinhole + Radial-Tangential undistort """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(z) == 2

  # Back project and undistort
  fx, fy, cx, cy = proj_params
  p = np.array([(z[0] - cx) / fx, (z[1] - cy) / fy])
  p_undist = radtan4_undistort(dist_params, p)

  # Project undistorted point to image plane
  return np.array([p_undist[0] * fx + cx, p_undist[1] * fy + cy])


def pinhole_radtan4_project_jacobian(proj_params, dist_params, p_C):
  """ Pinhole + Radial-Tangential project jacobian """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  # Project 3D point
  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])

  # Jacobian
  J_proj = zeros((2, 3))
  J_proj[0, :] = [1 / p_C[2], 0, -p_C[0] / p_C[2]**2]
  J_proj[1, :] = [0, 1 / p_C[2], -p_C[1] / p_C[2]**2]
  J_dist_point = radtan4_point_jacobian(dist_params, x)
  J_proj_point = pinhole_point_jacobian(proj_params)

  return J_proj_point @ J_dist_point @ J_proj


def pinhole_radtan4_params_jacobian(proj_params, dist_params, p_C):
  """ Pinhole + Radial-Tangential params jacobian """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])  # Project 3D point
  x_dist = radtan4_distort(dist_params, x)  # Distort point

  J_proj_point = pinhole_point_jacobian(proj_params)
  J_dist_params = radtan4_params_jacobian(dist_params, x)

  J = zeros((2, 8))
  J[0:2, 0:4] = pinhole_params_jacobian(x_dist)
  J[0:2, 4:8] = J_proj_point @ J_dist_params
  return J


# PINHOLE EQUI4 ###############################################################


def pinhole_equi4_project(proj_params, dist_params, p_C):
  """ Pinhole + Equi-distant project """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  # Project
  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])

  # Distort
  x_dist = equi4_distort(dist_params, x)

  # Scale and center to image plane
  fx, fy, cx, cy = proj_params
  z = np.array([fx * x_dist[0] + cx, fy * x_dist[1] + cy])
  return z


def pinhole_equi4_backproject(proj_params, dist_params, z):
  """ Pinhole + Equi-distant back-project """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(z) == 2

  # Convert image pixel coordinates to normalized retinal coordintes
  fx, fy, cx, cy = proj_params
  x = np.array([(z[0] - cx) / fx, (z[1] - cy) / fy, 1.0])

  # Undistort
  x = equi4_undistort(dist_params, x)

  # 3D ray
  p = np.array([x[0], x[1], 1.0])
  return p


def pinhole_equi4_undistort(proj_params, dist_params, z):
  """ Pinhole + Equi-distant undistort """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(z) == 2

  # Back project and undistort
  fx, fy, cx, cy = proj_params
  p = np.array([(z[0] - cx) / fx, (z[1] - cy) / fy])
  p_undist = equi4_undistort(dist_params, p)

  # Project undistorted point to image plane
  return np.array([p_undist[0] * fx + cx, p_undist[1] * fy + cy])


def pinhole_equi4_project_jacobian(proj_params, dist_params, p_C):
  """ Pinhole + Equi-distant project jacobian """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  # Project 3D point
  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])

  # Jacobian
  J_proj = zeros((2, 3))
  J_proj[0, :] = [1 / p_C[2], 0, -p_C[0] / p_C[2]**2]
  J_proj[1, :] = [0, 1 / p_C[2], -p_C[1] / p_C[2]**2]
  J_dist_point = equi4_point_jacobian(dist_params, x)
  J_proj_point = pinhole_point_jacobian(proj_params)
  return J_proj_point @ J_dist_point @ J_proj


def pinhole_equi4_params_jacobian(proj_params, dist_params, p_C):
  """ Pinhole + Equi-distant params jacobian """
  assert len(proj_params) == 4
  assert len(dist_params) == 4
  assert len(p_C) == 3

  x = np.array([p_C[0] / p_C[2], p_C[1] / p_C[2]])  # Project 3D point
  x_dist = equi4_distort(dist_params, x)  # Distort point

  J_proj_point = pinhole_point_jacobian(proj_params)
  J_dist_params = equi4_params_jacobian(dist_params, x)

  J = zeros((2, 8))
  J[0:2, 0:4] = pinhole_params_jacobian(x_dist)
  J[0:2, 4:8] = J_proj_point @ J_dist_params
  return J


# CAMERA GEOMETRY #############################################################


@dataclass
class CameraGeometry:
  """ Camera Geometry """
  cam_idx: int
  resolution: tuple
  proj_model: str
  dist_model: str
  proj_params_size: int
  dist_params_size: int

  project_fn: FunctionType
  backproject_fn: FunctionType
  undistort_fn: FunctionType
  J_proj_fn: FunctionType
  J_params_fn: FunctionType

  def proj_params(self, params):
    """ Extract projection parameters """
    return params[:self.proj_params_size]

  def dist_params(self, params):
    """ Extract distortion parameters """
    return params[-self.dist_params_size:]

  def project(self, params, p_C):
    """ Project point `p_C` with camera parameters `params` """
    proj_params = params[:self.proj_params_size]
    dist_params = params[-self.dist_params_size:]
    return self.project_fn(proj_params, dist_params, p_C)

  def backproject(self, params, z):
    """ Back-project image point `z` with camera parameters `params` """
    proj_params = params[:self.proj_params_size]
    dist_params = params[-self.dist_params_size:]
    return self.project_fn(proj_params, dist_params, z)

  def undistort(self, params, z):
    """ Undistort image point `z` with camera parameters `params` """
    proj_params = params[:self.proj_params_size]
    dist_params = params[-self.dist_params_size:]
    return self.undistort_fn(proj_params, dist_params, z)

  def J_proj(self, params, p_C):
    """ Form Jacobian w.r.t. p_C """
    proj_params = params[:self.proj_params_size]
    dist_params = params[-self.dist_params_size:]
    return self.J_proj_fn(proj_params, dist_params, p_C)

  def J_params(self, params, p_C):
    """ Form Jacobian w.r.t. camera parameters """
    proj_params = params[:self.proj_params_size]
    dist_params = params[-self.dist_params_size:]
    return self.J_params_fn(proj_params, dist_params, p_C)


def pinhole_radtan4_setup(cam_idx, cam_res):
  """ Setup Pinhole + Radtan4 camera geometry """
  return CameraGeometry(
      cam_idx, cam_res, "pinhole", "radtan4", 4, 4, pinhole_radtan4_project,
      pinhole_radtan4_backproject, pinhole_radtan4_undistort,
      pinhole_radtan4_project_jacobian, pinhole_radtan4_params_jacobian)


def pinhole_equi4_setup(cam_idx, cam_res):
  """ Setup Pinhole + Equi camera geometry """
  return CameraGeometry(cam_idx, cam_res, "pinhole", "equi4", 4, 4,
                        pinhole_equi4_project, pinhole_equi4_backproject,
                        pinhole_equi4_undistort, pinhole_equi4_project_jacobian,
                        pinhole_equi4_params_jacobian)


def camera_geometry_setup(cam_idx, cam_res, proj_model, dist_model):
  """ Setup camera geometry """
  if proj_model == "pinhole" and dist_model == "radtan4":
    return pinhole_radtan4_setup(cam_idx, cam_res)
  elif proj_model == "pinhole" and dist_model == "equi4":
    return pinhole_radtan4_setup(cam_idx, cam_res)
  else:
    raise RuntimeError(f"Unrecognized [{proj_model}]-[{dist_model}] combo!")


###############################################################################
# STATE ESTIMATION
###############################################################################

# STATE VARIABLES #############################################################


@dataclass
class StateVariable:
  """ State variable """
  ts: int
  var_type: str
  param: np.array
  parameterization: str
  min_dims: int
  fix: bool
  data: Optional[dict] = None


class StateVariableType(Enum):
  """ State Variable Type """
  POSE = 1
  EXTRINSICS = 2
  FEATURE = 3
  CAMERA = 4
  SPEED_AND_BIASES = 5


def tf2pose(T):
  """ Form pose vector """
  rx, ry, rz = tf_trans(T)
  qw, qx, qy, qz = tf_quat(T)
  return np.array([rx, ry, rz, qx, qy, qz, qw])


def pose2tf(pose_vec):
  """ Convert pose vector to transformation matrix """
  rx, ry, rz = pose_vec[0:3]
  qx, qy, qz, qw = pose_vec[3:7]
  return tf(np.array([qw, qx, qy, qz]), np.array([rx, ry, rz]))


def pose_setup(ts, param, fix=False):
  """ Forms pose state-variable """
  param = tf2pose(param) if param.shape == (4, 4) else param
  return StateVariable(ts, "pose", param, None, 6, fix)


def extrinsics_setup(param, fix=False):
  """ Forms extrinsics state-variable """
  param = tf2pose(param) if param.shape == (4, 4) else param
  return StateVariable(None, "extrinsics", param, None, 6, fix)


def camera_params_setup(cam_idx, res, proj_model, dist_model, param, fix=False):
  """ Forms camera parameters state-variable """
  data = camera_geometry_setup(cam_idx, res, proj_model, dist_model)
  return StateVariable(None, "camera", param, None, len(param), fix, data)


def feature_setup(param, fix=False):
  """ Forms feature state-variable """
  return StateVariable(None, "feature", param, None, len(param), fix)


def speed_biases_setup(ts, vel, ba, bg, fix=False):
  """ Forms speed and biases state-variable """
  param = np.block([vel, ba, bg])
  return StateVariable(ts, "speed_and_biases", param, None, len(param), fix)


def perturb_state_variable(sv, i, step_size):
  """ Perturb state variable """
  if sv.var_type == "pose" or sv.var_type == "extrinsics":
    T = pose2tf(sv.param)
    T_dash = tf_perturb(T, i, step_size)
    sv.param = tf2pose(T_dash)
  else:
    sv.param[i] += step_size

  return sv


def update_state_variable(sv, dx):
  """ Update state variable """
  if sv.var_type == "pose" or sv.var_type == "extrinsics":
    T = pose2tf(sv.param)
    T_prime = tf_update(T, dx)
    sv.param = tf2pose(T_prime)
  else:
    sv.param += dx


# FACTORS ######################################################################


@dataclass
class Factor:
  """ Factor """
  factor_type: str
  param_ids: str
  measurement: np.array
  covar: np.array
  eval_fn: FunctionType
  data: Optional[dict] = None
  sqrt_info: np.array = None

  def __post_init__(self):
    self.sqrt_info = chol(inv(self.covar))

  def eval(self, params):
    """ Evaluate factor """
    return self.eval_fn(self, params)


def pose_factor_setup(param_ids, measurement, covar=eye(6)):
  """ Setup Pose Factor """
  assert len(param_ids) == 1
  assert measurement.shape == (4, 4)
  assert covar.shape == (6, 6)

  # Pose factor eval function
  def pose_factor_eval(factor, params):
    """ Evaluate pose factor """
    # Measured pose
    T_meas = factor.measurement
    q_meas = tf_quat(T_meas)
    r_meas = tf_trans(T_meas)

    # Estimated pose
    pose_est = params[0]
    T_est = pose2tf(pose_est.param)
    q_est = tf_quat(T_est)
    r_est = tf_trans(T_est)

    # Form residuals (pose - pose_est)
    dr = r_meas - r_est
    dq = quat_mul(quat_inv(q_meas), q_est)
    dtheta = 2 * dq[1:4]
    r = factor.sqrt_info @ np.block([dr, dtheta])

    # Form jacobians
    J = zeros((6, 6))
    J[0:3, 0:3] = -eye(3)
    J[3:6, 3:6] = quat_left(dq)[1:4, 1:4]
    J = factor.sqrt_info @ J
    return (r, [J])

  return Factor("pose_factor", param_ids, measurement, covar, pose_factor_eval)


def ba_factor_setup(param_ids, z, cam_geom, covar=eye(2)):
  """ Setup BA factor """
  assert len(param_ids) == 3
  assert len(z) == 2
  assert covar.shape == (2, 2)

  # BA factor eval function
  def ba_factor_eval(factor, params):
    """ Evaluate bundle adjustment factor """
    assert factor is not None
    assert len(params) == 3

    # Map params
    cam_pose, feature, cam_params = params

    # Project point in world frame to image plane
    cam_geom = factor.data['cam_geom']
    T_WC = pose2tf(cam_pose.param)
    z_hat = zeros((2, 1))
    p_W = zeros((3, 1))
    p_W = feature.param
    p_C = tf_point(inv(T_WC), p_W)
    z_hat = cam_geom.project(cam_params.param, p_C)

    # Calculate residual
    sqrt_info = factor.sqrt_info
    z = factor.measurement
    r = sqrt_info @ (z - z_hat)

    # Calculate Jacobians
    # -- Measurement model jacobian
    neg_sqrt_info = -1.0 * sqrt_info
    Jh = cam_geom.J_proj(cam_params.param, p_C)
    Jh_weighted = neg_sqrt_info @ Jh
    # -- Jacobian w.r.t. camera pose T_WC
    C_WC = tf_rot(T_WC)
    C_CW = C_WC.T
    r_WC = tf_trans(T_WC)
    J0 = zeros((2, 6))  # w.r.t Camera pose T_WC
    J0[0:2, 0:3] = Jh_weighted @ -C_CW
    J0[0:2, 3:6] = Jh_weighted @ -C_CW @ skew(p_W - r_WC) @ -C_WC
    # -- Jacobian w.r.t. feature
    J1 = None
    J1 = zeros((2, 3))
    J1 = Jh_weighted @ C_CW
    # -- Jacobian w.r.t. camera parameters
    J_cam_params = cam_geom.J_params(cam_params.param, p_C)
    J2 = zeros((2, 8))
    J2 = neg_sqrt_info @ J_cam_params

    return (r, [J0, J1, J2])

  # Return ba factor
  data = {'cam_geom': cam_geom}
  return Factor("ba_factor", param_ids, z, covar, ba_factor_eval, data)


def vision_factor_setup(param_ids, z, cam_geom, covar=eye(2)):
  """ Form vision factor """
  assert len(param_ids) == 4
  assert len(z) == 2
  assert covar.shape == (2, 2)

  # Vision factor eval function
  def vision_factor_eval(factor, params):
    """ Evaluate vision factor """
    assert factor is not None
    assert len(params) == 4

    # Map params
    pose, cam_exts, feature, cam_params = params

    # Project point in world frame to image plane
    cam_geom = factor.data['cam_geom']
    T_WB = pose2tf(pose.param)
    T_BCi = pose2tf(cam_exts.param)
    p_W = feature.param
    p_C = tf_point(inv(T_WB @ T_BCi), p_W)
    z_hat = cam_geom.project(cam_params.param, p_C)

    # Calculate residual
    sqrt_info = factor.sqrt_info
    z = factor.measurement
    r = sqrt_info @ (z - z_hat)

    # Calculate Jacobians
    C_BCi = tf_rot(T_BCi)
    C_WB = tf_rot(T_WB)
    C_CB = C_BCi.T
    C_BW = C_WB.T
    C_CW = C_CB @ C_WB.T
    r_WB = tf_trans(T_WB)
    neg_sqrt_info = -1.0 * sqrt_info
    # -- Measurement model jacobian
    Jh = cam_geom.J_proj(cam_params.param, p_C)
    Jh_weighted = neg_sqrt_info @ Jh
    # -- Jacobian w.r.t. pose T_WB
    J0 = zeros((2, 6))
    J0[0:2, 0:3] = Jh_weighted @ C_CB @ -C_BW
    J0[0:2, 3:6] = Jh_weighted @ C_CB @ -C_BW @ skew(p_W - r_WB) @ -C_WB
    # -- Jacobian w.r.t. camera extrinsics T_BCi
    J1 = zeros((2, 6))
    J1[0:2, 0:3] = Jh_weighted @ -C_CB
    J1[0:2, 3:6] = Jh_weighted @ -C_CB @ skew(C_BCi @ p_C) @ -C_BCi
    # -- Jacobian w.r.t. feature
    J2 = zeros((2, 3))
    J2 = Jh_weighted @ C_CW
    # -- Jacobian w.r.t. camera parameters
    J_cam_params = cam_geom.J_params(cam_params.param, p_C)
    J3 = zeros((2, 8))
    J3 = neg_sqrt_info @ J_cam_params

    return (r, [J0, J1, J2, J3])

  # Return vision factor
  data = {'cam_geom': cam_geom}
  return Factor("vision_factor", param_ids, z, covar, vision_factor_eval, data)


@dataclass
class ImuBuffer:
  """ IMU buffer """
  ts: List[int]
  acc: List[np.array]
  gyr: List[np.array]


@dataclass
class ImuParams:
  """ IMU parameters """
  noise_acc: np.array
  noise_gyr: np.array
  noise_ba: np.array
  noise_bg: np.array
  g: np.array = np.array([0.0, 0.0, 9.81])


@dataclass
class ImuFactorData:
  """ IMU Factor data """
  state_F: np.array
  state_P: np.array
  dr: np.array
  dv: np.array
  dC: np.array
  ba: np.array
  bg: np.array
  g: np.array
  Dt: float


def imu_factor_propagate(imu_buf, imu_params, sb_i):
  """ IMU factor propagate """
  # Setup
  Dt = 0.0
  g = imu_params.g
  state_F = eye(15, 15)  # State jacobian
  state_P = zeros((15, 15))  # State covariance

  # Noise matrix Q
  Q = zeros((12, 12))
  Q[0:3, 0:3] = imu_params.noise_acc**2 * eye(3)
  Q[3:6, 3:6] = imu_params.noise_gyr**2 * eye(3)
  Q[6:9, 6:9] = imu_params.noise_ba**2 * eye(3)
  Q[9:12, 9:12] = imu_params.noise_bg**2 * eye(3)

  # Pre-integrate relative position, velocity, rotation and biases
  dr = np.array([0.0, 0.0, 0.0])  # Relative position
  dv = np.array([0.0, 0.0, 0.0])  # Relative velocity
  dC = eye(3)  # Relative rotation
  ba_i = sb_i.param[3:6]  # Accel biase at i
  bg_i = sb_i.param[6:9]  # Gyro biase at i

  # Pre-integrate imu measuremenets
  for k in range(len(imu_buf.ts) - 1):
    # Timestep
    ts_i = imu_buf.ts[k]
    ts_j = imu_buf.ts[k + 1]
    dt = ts2sec(ts_j - ts_i)
    dt_sq = dt * dt

    # Accelerometer and gyroscope measurements
    acc_i = imu_buf.acc[k]
    gyr_i = imu_buf.gyr[k]

    # Propagate IMU state using Euler method
    dr = dr + (dv * dt) + (0.5 * dC @ (acc_i - ba_i) * dt_sq)
    dv = dv + dC @ (acc_i - ba_i) * dt
    dC = dC @ Exp((gyr_i - bg_i) * dt)
    ba = ba_i
    bg = bg_i

    # Make sure determinant of rotation is 1 by normalizing the quaternion
    dq = quat_normalize(rot2quat(dC))
    dC = quat2rot(dq)

    # Continuous time transition matrix F
    F = zeros((15, 15))
    F[0:3, 3:6] = eye(3)
    F[3:6, 6:9] = -1.0 * dC @ skew(acc_i - ba_i)
    F[3:6, 9:12] = -1.0 * dC
    F[6:9, 6:9] = -1.0 * skew(gyr_i - bg_i)
    F[6:9, 12:15] = -eye(3)

    # Continuous time input jacobian G
    G = zeros((15, 12))
    G[3:6, 0:3] = -1.0 * dC
    G[6:9, 3:6] = -eye(3)
    G[9:12, 6:9] = eye(3)
    G[12:15, 9:12] = eye(3)

    # Update
    G_dt = G * dt
    I_F_dt = eye(15) + F * dt
    state_F = I_F_dt @ state_F
    state_P = I_F_dt @ state_P @ I_F_dt.T + G_dt @ Q @ G_dt.T
    Dt += dt

  # Update
  return ImuFactorData(state_F, state_P, dr, dv, dC, ba, bg, g, Dt)


def imu_factor_eval(factor, params):
  """ Evaluate IMU factor """
  # Map params
  pose_i, sb_i, pose_j, sb_j = params

  # Timestep i
  T_i = pose2tf(pose_i.param)
  r_i = tf_trans(T_i)
  C_i = tf_rot(T_i)
  q_i = tf_quat(T_i)
  v_i = sb_i.param[0:3]
  ba_i = sb_i.param[3:6]
  bg_i = sb_i.param[6:9]

  # Timestep j
  T_j = pose2tf(pose_j.param)
  r_j = tf_trans(T_j)
  C_j = tf_rot(T_j)
  q_j = tf_quat(T_j)
  v_j = sb_j.param[0:3]

  # Correct the relative position, velocity and orientation
  # -- Extract jacobians from error-state jacobian
  dr_dba = factor.data.state_F[0:3, 9:12]
  dr_dbg = factor.data.state_F[0:3, 12:15]
  dv_dba = factor.data.state_F[3:6, 9:12]
  dv_dbg = factor.data.state_F[3:6, 12:15]
  dq_dbg = factor.data.state_F[6:9, 12:15]
  dba = ba_i - factor.data.ba
  dbg = bg_i - factor.data.bg
  # -- Correct the relative position, velocity and rotation
  dr = factor.data.dr + dr_dba @ dba + dr_dbg @ dbg
  dv = factor.data.dv + dv_dba @ dba + dv_dbg @ dbg
  dC = factor.data.dC @ Exp(dq_dbg @ dbg)
  dq = quat_normalize(rot2quat(dC))

  # Form residuals
  sqrt_info = factor.sqrt_info
  g = factor.data.g
  Dt = factor.data.Dt
  Dt_sq = Dt * Dt

  dr_meas = (C_i.T @ ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq)))
  dv_meas = (C_i.T @ ((v_j - v_i) + (g * Dt)))

  err_pos = dr_meas - dr
  err_vel = dv_meas - dv
  err_rot = (2.0 * quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i), q_j)))[1:4]
  err_ba = np.array([0.0, 0.0, 0.0])
  err_bg = np.array([0.0, 0.0, 0.0])
  r = sqrt_info @ np.block([err_pos, err_vel, err_rot, err_ba, err_bg])

  # Form jacobians
  J0 = zeros((15, 6))  # residuals w.r.t pose i
  J1 = zeros((15, 9))  # residuals w.r.t speed and biase i
  J2 = zeros((15, 6))  # residuals w.r.t pose j
  J3 = zeros((15, 9))  # residuals w.r.t speed and biase j

  # -- Jacobian w.r.t. pose i
  # yapf: disable
  J0[0:3, 0:3] = -C_i.T  # dr w.r.t r_i
  J0[0:3, 3:6] = skew(dr_meas)  # dr w.r.t C_i
  J0[3:6, 3:6] = skew(dv_meas)  # dv w.r.t C_i
  J0[6:9, 3:6] = -(quat_left(rot2quat(C_j.T @ C_i)) @ quat_right(dq))[1:4, 1:4]  # dtheta w.r.t C_i
  J0 = sqrt_info @ J0
  # yapf: enable

  # -- Jacobian w.r.t. speed and biases i
  # yapf: disable
  J1[0:3, 0:3] = -C_i.T * Dt  # dr w.r.t v_i
  J1[0:3, 3:6] = -dr_dba  # dr w.r.t ba
  J1[0:3, 6:9] = -dr_dbg  # dr w.r.t bg
  J1[3:6, 0:3] = -C_i.T  # dv w.r.t v_i
  J1[3:6, 3:6] = -dv_dba  # dv w.r.t ba
  J1[3:6, 6:9] = -dv_dbg  # dv w.r.t bg
  J1[6:9, 6:9] = -quat_left(rot2quat(C_j.T @ C_i @ factor.data.dC))[1:4, 1:4] @ dq_dbg  # dtheta w.r.t C_i
  J1 = sqrt_info @ J1
  # yapf: enable

  # -- Jacobian w.r.t. pose j
  # yapf: disable
  J2[0:3, 0:3] = C_i.T  # dr w.r.t r_j
  J2[6:9, 3:6] = quat_left(rot2quat(dC.T @ C_i.T @ C_j))[1:4, 1:4]  # dtheta w.r.t C_j
  J2 = sqrt_info @ J2
  # yapf: enable

  # -- Jacobian w.r.t. sb j
  J3[3:6, 0:3] = C_i.T  # dv w.r.t v_j
  J3 = sqrt_info @ J3

  return (r, [J0, J1, J2, J3])


def imu_factor_setup(param_ids, imu_buf, imu_params, sb_i):
  """ Setup IMU factor """
  assert len(param_ids) == 4
  data = imu_factor_propagate(imu_buf, imu_params, sb_i)
  covar = pinv(data.state_P)
  return Factor("imu_factor", param_ids, None, covar, imu_factor_eval, data)


def check_factor_jacobian(factor, params, param_idx, jac_name, **kwargs):
  """ Check factor jacobian """

  # Step size and threshold
  step_size = kwargs.get('step_size', 1e-8)
  threshold = kwargs.get('threshold', 1e-4)
  verbose = kwargs.get('verbose', False)

  # Calculate baseline
  r, jacs = factor.eval(params)

  # Numerical diff
  J_fdiff = zeros((len(r), params[param_idx].min_dims))
  for i in range(params[param_idx].min_dims):
    # Forward difference and evaluate
    params_fwd = copy.deepcopy(params)
    param = params_fwd[param_idx]
    params_fwd[param_idx] = perturb_state_variable(param, i, 0.5 * step_size)
    r_fwd, _ = factor.eval(params_fwd)

    # Backward difference and evaluate
    params_bwd = copy.deepcopy(params)
    param = params_bwd[param_idx]
    params_bwd[param_idx] = perturb_state_variable(param, i, -0.5 * step_size)
    r_bwd, _ = factor.eval(params_bwd)

    # Central finite difference
    J_fdiff[:, i] = (r_fwd - r_bwd) / step_size

  J = jacs[param_idx]
  return check_jacobian(jac_name, J_fdiff, J, threshold, verbose)


# FACTOR GRAPH ################################################################


class FactorGraph:
  """ Factor Graph """

  def __init__(self):
    # Parameters and factors
    self.params = {}
    self.factors = {}

    # Solver
    self.solver_max_iter = 5
    # self.solver_lambda = 1e-4
    self.solver_lambda = 1e-4

  def add_param(self, param):
    """ Add param """
    # Add param
    param_id = len(self.params)
    self.params[param_id] = param
    return param_id

  def add_factor(self, factor):
    """ Add factor """
    # Double check if params exists
    for param_id in factor.param_ids:
      if param_id not in self.params:
        raise RuntimeError(f"Parameter [{param_id}] does not exist!")

    # Add factor
    factor_id = len(self.factors)
    self.factors[factor_id] = factor
    return factor_id

  def _get_factor_params(self, factor):
    """ Get factor parameters """
    return [self.params[param_id] for param_id in factor.param_ids]

  def _get_reproj_errors(self):
    """ Get reprojection errors """
    target_factors = ["ba_factor", "vision_factor"]

    reproj_errors = []
    for _, factor in self.factors.items():
      if factor.factor_type in target_factors:
        factor_params = self._get_factor_params(factor)
        r, _ = factor.eval(factor_params)
        reproj_errors.append(norm(r))

    return np.array(reproj_errors)

  def _print_to_console(self, iter_k, r):
    """ Print to console """
    # rmse_vision = rmse(self._get_reproj_errors())

    print(f"iter[{iter_k}]: ", end="")
    # print(f"rms_reproj_error: {rmse_vision:.2f} px", end=", ")
    print(f"cost: {self._cost(r):.2e}")
    sys.stdout.flush()

  def _form_param_indices(self):
    """ Form parameter indices """
    # Parameter ids
    pose_param_ids = set()
    sb_param_ids = set()
    camera_param_ids = set()
    exts_param_ids = set()
    feature_param_ids = set()

    # Track parameters
    nb_params = 0
    for _, factor in self.factors.items():
      for _, param_id in enumerate(factor.param_ids):
        param = self.params[param_id]

        if param.fix:
          continue
        elif param.var_type == "pose":
          pose_param_ids.add(param_id)
        elif param.var_type == "speed_and_biases":
          sb_param_ids.add(param_id)
        elif param.var_type == "extrinsics":
          exts_param_ids.add(param_id)
        elif param.var_type == "feature":
          feature_param_ids.add(param_id)
        elif param.var_type == "camera":
          camera_param_ids.add(param_id)
        nb_params += 1

    # Assign global parameter order
    param_ids_list = []
    param_ids_list.append(pose_param_ids)
    param_ids_list.append(sb_param_ids)
    param_ids_list.append(exts_param_ids)
    param_ids_list.append(feature_param_ids)
    param_ids_list.append(camera_param_ids)

    param_idxs = {}
    param_size = 0
    for param_ids in param_ids_list:
      for param_id in param_ids:
        param_idxs[param_id] = param_size
        param_size += self.params[param_id].min_dims

    return (param_idxs, param_size)

  def _form_hessian(self, param_idxs, param_size):
    """ Form Hessian matrix H """
    H = zeros((param_size, param_size))
    g = zeros(param_size)
    residuals = []

    for _, factor in self.factors.items():
      factor_param_ids = factor.param_ids
      factor_params = [self.params[param_id] for param_id in factor_param_ids]
      r, jacobians = factor.eval(factor_params)
      residuals.append(r)

      nb_params = len(factor_params)
      for i in range(nb_params):
        param_i = self.params[factor_param_ids[i]]
        if param_i.fix:
          continue
        idx_i = param_idxs[factor_param_ids[i]]
        size_i = param_i.min_dims
        J_i = jacobians[i]

        for j in range(i, nb_params):
          param_j = self.params[factor_param_ids[j]]
          if param_j.fix:
            continue
          idx_j = param_idxs[factor_param_ids[j]]
          size_j = param_j.min_dims
          J_j = jacobians[j]

          rs = idx_i
          re = idx_i + size_i
          cs = idx_j
          ce = idx_j + size_j

          if i == j:
            # Diagonal
            H[rs:re, cs:ce] += J_i.T @ J_j
          else:
            # Off-Diagonal
            H[rs:re, cs:ce] += J_i.T @ J_j
            H[cs:ce, rs:re] += H[rs:re, cs:ce].T

        rs = idx_i
        re = idx_i + size_i
        g[rs:re] += (-J_i.T @ r)

    return (H, g, np.array(r))

  def _evaluate(self):
    """ Evaluate """
    (param_idxs, param_size) = self._form_param_indices()
    (H, g, r) = self._form_hessian(param_idxs, param_size)
    return (param_idxs, H, g, r)

  def _update(self, param_idxs, dx):
    """ Update """
    for param_id, param in self.params.items():
      # Check if param even exists
      if param_id not in param_idxs:
        continue

      # Update parameter
      start = param_idxs[param_id]
      end = start + param.min_dims
      param_dx = dx[start:end]
      update_state_variable(param, param_dx)

  @staticmethod
  def _cost(r):
    """ Cost """
    return 0.5 * r.T @ r

  def solve(self):
    """ Solve """
    lambda_k = self.solver_lambda
    (param_idxs, H, g, r) = self._evaluate()

    print(f"nb_factors: {len(self.factors)}")
    print(f"nb_params: {len(self.params)}")
    self._print_to_console(0, r)

    for i in range(1, self.solver_max_iter):
      # H = H + lambda_k * eye(H.shape[0])
      # dx = pinv(H) @ g
      # dx = np.linalg.solve(H, g)

      # sH = scipy.sparse.csc_matrix(H)
      # dx = scipy.sparse.linalg.spsolve(sH, g)

      H = H + lambda_k * eye(H.shape[0])
      c, low = scipy.linalg.cho_factor(H)
      dx = scipy.linalg.cho_solve((c, low), g)

      self._update(param_idxs, dx)
      (param_idxs, H, g, r) = self._evaluate()
      self._print_to_console(i, r)


# FEATURE TRACKING #############################################################


def draw_matches(img_i, img_j, kps_i, kps_j, **kwargs):
  """
  Draw keypoint matches between images `img_i` and `img_j` with keypoints
  `kps_i` and `kps_j`
  """
  assert len(kps_i) == len(kps_j)

  nb_kps = len(kps_i)
  viz = cv2.hconcat([img_i, img_j])
  viz = cv2.cvtColor(viz, cv2.COLOR_GRAY2RGB)

  color = (0, 255, 0)
  radius = 3
  thickness = kwargs.get('thickness', cv2.FILLED)
  linetype = kwargs.get('linetype', cv2.LINE_AA)

  for n in range(nb_kps):
    pt_i = None
    pt_j = None
    if hasattr(kps_i[n], 'pt'):
      pt_i = (int(kps_i[n].pt[0]), int(kps_i[n].pt[1]))
      pt_j = (int(kps_j[n].pt[0] + img_i.shape[1]), int(kps_j[n].pt[1]))
    else:
      pt_i = (int(kps_i[n][0]), int(kps_i[n][1]))
      pt_j = (int(kps_j[n][0] + img_i.shape[1]), int(kps_j[n][1]))

    cv2.circle(viz, pt_i, radius, color, thickness, lineType=linetype)
    cv2.circle(viz, pt_j, radius, color, thickness, lineType=linetype)
    cv2.line(viz, pt_i, pt_j, color, 1, linetype)

  return viz


def draw_keypoints(img, kps, inliers=None, **kwargs):
  """
  Draw points `kps` on image `img`. The `inliers` boolean list is optional
  and is expected to be the same size as `kps` denoting whether the point
  should be drawn or not.
  """
  inliers = [1 for i in range(len(kps))] if inliers is None else inliers
  radius = kwargs.get('radius', 2)
  color = kwargs.get('color', (0, 255, 0))
  thickness = kwargs.get('thickness', cv2.FILLED)
  linetype = kwargs.get('linetype', cv2.LINE_AA)

  viz = img
  if len(img.shape) == 2:
    viz = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

  for n, kp in enumerate(kps):
    if inliers[n]:
      p = (int(kp.pt[0]), int(kp.pt[1])) if hasattr(kp, 'pt') else kp
      cv2.circle(viz, p, radius, color, thickness, lineType=linetype)

  return viz


def sort_keypoints(kps):
  """ Sort a list of cv2.KeyPoint based on their response """
  responses = [kp.response for kp in kps]
  indices = range(len(responses))
  indices = sorted(indices, key=lambda i: responses[i], reverse=True)
  return [kps[i] for i in indices]


def spread_keypoints(img, kps, min_dist, **kwargs):
  """
  Given a set of keypoints `kps` make sure they are atleast `min_dist` pixels
  away from each other, if they are not remove them.
  """
  # Pre-check
  if not kps:
    return kps

  # Setup
  debug = kwargs.get('debug', False)
  prev_kps = kwargs.get('prev_kps', [])
  min_dist = int(min_dist)
  img_h, img_w = img.shape
  A = np.zeros(img.shape)  # Allowable areas are marked 0 else not allowed

  # Loop through previous keypoints
  for kp in prev_kps:
    # Convert from keypoint to tuple
    p = (int(kp.pt[0]), int(kp.pt[1]))

    # Fill the area of the matrix where the next keypoint cannot be around
    rs = int(max(p[1] - min_dist, 0.0))
    re = int(min(p[1] + min_dist + 1, img_h))
    cs = int(max(p[0] - min_dist, 0.0))
    ce = int(min(p[0] + min_dist + 1, img_w))
    A[rs:re, cs:ce] = np.ones((re - rs, ce - cs))

  # Loop through keypoints
  kps_results = []
  for kp in sort_keypoints(kps):
    # Convert from keypoint to tuple
    p = (int(kp.pt[0]), int(kp.pt[1]))

    # Check if point is ok to be added to results
    if A[p[1], p[0]] > 0.0:
      continue

    # Fill the area of the matrix where the next keypoint cannot be around
    rs = int(max(p[1] - min_dist, 0.0))
    re = int(min(p[1] + min_dist + 1, img_h))
    cs = int(max(p[0] - min_dist, 0.0))
    ce = int(min(p[0] + min_dist + 1, img_w))
    A[rs:re, cs:ce] = np.ones((re - rs, ce - cs))
    A[p[1], p[0]] = 2

    # Add to results
    kps_results.append(kp)

  # Debug
  if debug:
    img = draw_keypoints(img, kps_results, radius=3)

    plt.figure()

    ax = plt.subplot(121)
    ax.imshow(A)
    ax.set_xlabel('pixel')
    ax.set_ylabel('pixel')
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')

    ax = plt.subplot(122)
    ax.imshow(img)
    ax.set_xlabel('pixel')
    ax.set_ylabel('pixel')
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')

    plt.show()

  return kps_results


class FeatureGrid:
  """
  FeatureGrid

  The idea is to take all the feature positions and put them into grid cells
  across the full image space. This is so that one could keep track of how many
  feautures are being tracked in each individual grid cell and act accordingly.

  o-----> x
  | ---------------------
  | |  0 |  1 |  2 |  3 |
  V ---------------------
  y |  4 |  5 |  6 |  7 |
    ---------------------
    |  8 |  9 | 10 | 11 |
    ---------------------
    | 12 | 13 | 14 | 15 |
    ---------------------

    grid_x = ceil((max(1, pixel_x) / img_w) * grid_cols) - 1.0
    grid_y = ceil((max(1, pixel_y) / img_h) * grid_rows) - 1.0
    cell_id = int(grid_x + (grid_y * grid_cols))

  """

  def __init__(self, grid_rows, grid_cols, image_shape, keypoints):
    assert len(image_shape) == 2
    self.grid_rows = grid_rows
    self.grid_cols = grid_cols
    self.image_shape = image_shape
    self.keypoints = keypoints

    self.cell = [0 for i in range(self.grid_rows * self.grid_cols)]
    for kp in keypoints:
      if hasattr(kp, 'pt'):
        # cv2.KeyPoint
        assert (kp.pt[0] >= 0 and kp.pt[0] <= image_shape[1])
        assert (kp.pt[1] >= 0 and kp.pt[1] <= image_shape[0])
        self.cell[self.cell_index(kp.pt)] += 1
      else:
        # Tuple
        assert (kp[0] >= 0 and kp[0] <= image_shape[1])
        assert (kp[1] >= 0 and kp[1] <= image_shape[0])
        self.cell[self.cell_index(kp)] += 1

  def cell_index(self, pt):
    """ Return cell index based on point `pt` """
    pixel_x, pixel_y = pt
    img_h, img_w = self.image_shape
    grid_x = math.ceil((max(1, pixel_x) / img_w) * self.grid_cols) - 1.0
    grid_y = math.ceil((max(1, pixel_y) / img_h) * self.grid_rows) - 1.0
    cell_id = int(grid_x + (grid_y * self.grid_cols))
    return cell_id

  def count(self, cell_idx):
    """ Return cell count """
    return self.cell[cell_idx]


def grid_detect(detector, image, **kwargs):
  """
  Detect features uniformly using a grid system.
  """
  optflow_mode = kwargs.get('optflow_mode', False)
  max_keypoints = kwargs.get('max_keypoints', 240)
  grid_rows = kwargs.get('grid_rows', 3)
  grid_cols = kwargs.get('grid_cols', 4)
  prev_kps = kwargs.get('prev_kps', [])
  if prev_kps is None:
    prev_kps = []

  # Calculate number of grid cells and max corners per cell
  image_height, image_width = image.shape
  dx = int(math.ceil(float(image_width) / float(grid_cols)))
  dy = int(math.ceil(float(image_height) / float(grid_rows)))
  nb_cells = grid_rows * grid_cols
  max_per_cell = math.floor(max_keypoints / nb_cells)

  # Detect corners in each grid cell
  feature_grid = FeatureGrid(grid_rows, grid_cols, image.shape, prev_kps)
  des_all = []
  kps_all = []

  cell_idx = 0
  for y in range(0, image_height, dy):
    for x in range(0, image_width, dx):
      # Make sure roi width and height are not out of bounds
      w = image_width - x if (x + dx > image_width) else dx
      h = image_height - y if (y + dy > image_height) else dy

      # Detect corners in grid cell
      cs, ce, rs, re = (x, x + w, y, y + h)
      roi_image = image[rs:re, cs:ce]

      kps = None
      des = None
      if optflow_mode:
        detector.setNonmaxSuppression(1)
        kps = detector.detect(roi_image)
        kps = sort_keypoints(kps)

      else:
        kps = detector.detect(roi_image, None)
        kps, des = detector.compute(roi_image, kps)

      # Offset keypoints
      cell_vacancy = max_per_cell - feature_grid.count(cell_idx)
      if cell_vacancy <= 0:
        continue

      limit = min(len(kps), cell_vacancy)
      for i in range(limit):
        kp = kps[i]
        kp.pt = (kp.pt[0] + x, kp.pt[1] + y)
        kps_all.append(kp)
        des_all.append(des[i, :] if optflow_mode is False else None)

      # Update cell_idx
      cell_idx += 1

  # Space out the keypoints
  kps_all = spread_keypoints(image, kps_all, 20, prev_kps=prev_kps)

  # Debug
  if kwargs.get('debug', False):
    # Setup
    viz = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    kps_grid = FeatureGrid(grid_rows, grid_cols, image.shape, kps_all)

    # Visualization properties
    red = (0, 0, 255)
    yellow = (0, 255, 255)
    linetype = cv2.LINE_AA
    font = cv2.FONT_HERSHEY_SIMPLEX

    # -- Draw horizontal lines
    for x in range(0, image_width, dx):
      cv2.line(viz, (x, 0), (x, image_height), red, 1, linetype)

    # -- Draw vertical lines
    for y in range(0, image_height, dy):
      cv2.line(viz, (0, y), (image_width, y), red, 1, linetype)

    # -- Draw bin numbers
    cell_idx = 0
    for y in range(0, image_height, dy):
      for x in range(0, image_width, dx):
        text = str(kps_grid.count(cell_idx))
        origin = (x + 10, y + 20)
        viz = cv2.putText(viz, text, origin, font, 0.5, red, 1, linetype)

        # text = str(feature_grid.count(cell_idx))
        # origin = (x + 10, y + 20)
        # viz = cv2.putText(viz, text, origin, font, 0.5, yellow, 1, linetype)

        cell_idx += 1

    # -- Draw keypoints
    viz = draw_keypoints(viz, kps_all, color=red)
    viz = draw_keypoints(viz, prev_kps, color=yellow)
    cv2.imshow("viz", viz)
    cv2.waitKey(0)

  # Return
  if optflow_mode:
    return kps_all

  return kps_all, np.array(des_all)


def optflow_track(img_i, img_j, pts_i, **kwargs):
  """
  Track keypoints `pts_i` from image `img_i` to image `img_j` using optical
  flow. Returns a tuple of `(pts_i, pts_j, inliers)` points in image i, j and a
  vector of inliers.
  """
  # Setup
  patch_size = kwargs.get('patch_size', 50)
  max_iter = kwargs.get('max_iter', 100)
  epsilon = kwargs.get('epsilon', 0.001)
  crit = (cv2.TermCriteria_COUNT | cv2.TermCriteria_EPS, max_iter, epsilon)

  # Optical flow settings
  config = {}
  config['winSize'] = (patch_size, patch_size)
  config['maxLevel'] = 3
  config['criteria'] = crit
  config['flags'] = cv2.OPTFLOW_USE_INITIAL_FLOW

  # Track using optical flow
  pts_j = np.array(pts_i)
  track_results = cv2.calcOpticalFlowPyrLK(img_i, img_j, pts_i, pts_j, **config)
  (pts_j, optflow_inliers, _) = track_results

  # Make sure keypoints are within image dimensions
  bound_inliers = []
  img_h, img_w = img_j.shape
  for p in pts_j:
    x_ok = p[0] >= 0 and p[0] <= img_w
    y_ok = p[1] >= 0 and p[1] <= img_h
    if x_ok and y_ok:
      bound_inliers.append(True)
    else:
      bound_inliers.append(False)

  # Update or mark feature as lost
  assert len(pts_i) == optflow_inliers.shape[0]
  assert len(pts_i) == len(bound_inliers)
  inliers = []
  for i in range(len(pts_i)):
    if optflow_inliers[i, 0] and bound_inliers[i]:
      inliers.append(True)
    else:
      inliers.append(False)

  if kwargs.get('debug', False):
    viz_i = draw_keypoints(img_i, pts_i, inliers)
    viz_j = draw_keypoints(img_j, pts_j, inliers)
    viz = cv2.hconcat([viz_i, viz_j])
    cv2.imshow('viz', viz)
    cv2.waitKey(0)

  return (pts_i, pts_j, inliers)


def filter_outliers(pts_i, pts_j, inliers):
  """ Filter outliers """
  pts_out_i = []
  pts_out_j = []
  for n, status in enumerate(inliers):
    if status:
      pts_out_i.append(pts_i[n])
      pts_out_j.append(pts_j[n])

  return (pts_out_i, pts_out_j)


def ransac(pts_i, pts_j, cam_i, cam_j):
  """ RANSAC """
  # Setup
  cam_geom_i = cam_i.data
  cam_geom_j = cam_j.data

  # Undistort points
  points_i = np.array([cam_geom_i.undistort(cam_i.param, p) for p in pts_i])
  points_j = np.array([cam_geom_j.undistort(cam_j.param, p) for p in pts_j])

  # Ransac via OpenCV's find fundamental matrix
  _, inliers = cv2.findFundamentalMat(points_i, points_j, cv2.FM_8POINT)

  return inliers.flatten()


@dataclass
class FeatureTrackingData:
  """
  Feature tracking data per camera

  This data structure keeps track of:

  - Image
  - Keypoints
  - Descriptors
  - Feature ids (optional)

  """
  cam_idx: int
  image: np.array
  keypoints: List[np.array]
  descriptors: np.array = None
  feature_ids: List[int] = None

  def get_all(self):
    """ Get all data """
    return (self.cam_idx, self.image, self.keypoints, self.descriptors,
            self.feature_ids)


class FeatureTracker:
  """ Feature tracker """

  def __init__(self):
    # Settings
    self.mode = "TRACK_DEFAULT"
    # self.mode = "TRACK_OVERLAPS"
    # self.mode = "TRACK_INDEPENDENT"

    # Settings
    self.reproj_threshold = 5.0

    # Data
    self.prev_ts = None
    self.frame_idx = 0
    self.detector = cv2.FastFeatureDetector_create(threshold=50)
    self.features_detected = 0
    self.features_tracking = 0
    self.feature_overlaps = {}
    self.prev_camera_images = None

    self.cam_idxs = []
    self.cam_params = {}
    self.cam_exts = {}
    self.cam_overlaps = []
    self.cam_data = {}

  def add_camera(self, cam_idx, cam_params, cam_exts):
    """ Add camera """
    self.cam_idxs.append(cam_idx)
    self.cam_data[cam_idx] = None
    self.cam_params[cam_idx] = cam_params
    self.cam_exts[cam_idx] = cam_exts

  def add_overlap(self, cam_i_idx, cam_j_idx):
    """ Add overlap """
    self.cam_overlaps.append((cam_i_idx, cam_j_idx))

  def num_tracking(self):
    """ Return number of features tracking """
    feature_ids = []
    for _, cam_data in self.cam_data.items():
      if cam_data is not None:
        feature_ids.extend(cam_data.feature_ids)
    return len(set(feature_ids))

  def _get_keypoints(self, cam_idx):
    """ Get keypoints observed by camera `cam_idx` """
    keypoints = None
    if self.cam_data[cam_idx] is not None:
      keypoints = self.cam_data[cam_idx].keypoints

    return keypoints

  def _get_feature_ids(self, cam_idx):
    """ Get feature ids observed by camera `cam_idx` """
    feature_ids = None
    if self.cam_data[cam_idx] is not None:
      feature_ids = self.cam_data[cam_idx].feature_ids

    return feature_ids

  def _form_feature_ids(self, nb_kps):
    """ Form list of feature ids for new features to be added """
    self.features_detected += nb_kps
    start_idx = self.features_detected - nb_kps
    end_idx = start_idx + nb_kps
    return list(range(start_idx, end_idx))

  def _triangulate(self, idx_i, idx_j, z_i, z_j):
    """ Triangulate feature """
    # Setup
    cam_i = self.cam_params[idx_i]
    cam_j = self.cam_params[idx_j]
    cam_geom_i = cam_i.data
    cam_geom_j = cam_j.data
    cam_exts_i = self.cam_exts[idx_i]
    cam_exts_j = self.cam_exts[idx_j]

    # Form projection matrices P_i and P_j
    T_BCi = pose2tf(cam_exts_i.param)
    T_BCj = pose2tf(cam_exts_j.param)
    T_CiCj = inv(T_BCi) @ T_BCj
    P_i = pinhole_P(cam_geom_i.proj_params(cam_i.param), eye(4))
    P_j = pinhole_P(cam_geom_j.proj_params(cam_i.param), T_CiCj)

    # Undistort image points z_i and z_j
    x_i = cam_geom_i.undistort(cam_i.param, z_i)
    x_j = cam_geom_j.undistort(cam_j.param, z_j)

    # Linear triangulate
    p_Ci = linear_triangulation(P_i, P_j, x_i, x_j)

    return p_Ci

  def _reproj_filter(self, idx_i, idx_j, pts_i, pts_j):
    """ Filter features by triangulating them via a stereo-pair and see if the
    reprojection error is reasonable """
    assert idx_i != idx_j
    assert len(pts_i) == len(pts_j)

    # Reject outliers based on reprojection error
    reproj_inliers = []
    cam_i = self.cam_params[idx_i]
    cam_geom_i = cam_i.data

    nb_pts = len(pts_i)
    for n in range(nb_pts):
      # Triangulate
      z_i = pts_i[n]
      z_j = pts_j[n]
      p_Ci = self._triangulate(idx_i, idx_j, z_i, z_j)
      if p_Ci[2] < 0.0:
        reproj_inliers.append(False)
        continue

      # Reproject
      z_i_hat = cam_geom_i.project(cam_i.param, p_Ci)
      reproj_error = norm(z_i - z_i_hat)
      if reproj_error > self.reproj_threshold:
        reproj_inliers.append(False)
        continue
      reproj_inliers.append(True)

    return reproj_inliers

  @staticmethod
  def _filter_outliers(cam_idxs, cam_pts, inliers, kp_size, feature_ids=None):
    """ Filter outliers """
    # Setup
    kps = {}
    f_ids = []

    # Loop through each camear
    for cam_idx in cam_idxs:
      kps[cam_idx] = []

      for n, p in enumerate(cam_pts[cam_idx]):
        # Check if inlier
        if not inliers[n]:
          continue

        # Process keypoint
        if hasattr(p, 'pt'):
          kps[cam_idx].append(p)  # cv2.KeyPoint already
        else:
          kps[cam_idx].append(cv2.KeyPoint(p[0], p[1], kp_size))

        # Feature id
        if feature_ids is not None:
          f_ids.append(feature_ids[n])

    # Return
    if feature_ids:
      return kps, f_ids

    return kps

  def _add_features(self, cam_idxs, cam_imgs, cam_pts, kp_size, inliers=None):
    """ Add features """
    # Pre-check
    assert cam_idxs
    assert all(cam_idx in cam_imgs for cam_idx in cam_idxs)
    assert all(cam_idx in cam_pts for cam_idx in cam_idxs)

    # Check if inliers is set else initialize it
    if inliers is None:
      inliers = [1 for i, _ in enumerate(cam_pts[cam_idxs[0]])]

    # Filter outliers
    cam_kps = self._filter_outliers(cam_idxs, cam_pts, inliers, kp_size)

    # Add camera data
    nb_inliers = int(np.sum(inliers))
    fids = self._form_feature_ids(nb_inliers)
    for idx in cam_idxs:
      img = cam_imgs[idx]
      kps = cam_kps[idx]

      if self.cam_data[idx] is None:
        ft_data = FeatureTrackingData(idx, img, kps, None, fids)
        self.cam_data[idx] = ft_data
      else:
        self.cam_data[idx].image = img
        self.cam_data[idx].keypoints.extend(kps)
        self.cam_data[idx].feature_ids.extend(fids)

    # Update overlapping features
    if len(cam_idxs) > 1:
      for fid in fids:
        self.feature_overlaps[fid] = 2

  def _update_features(self, cam_idxs, cam_imgs, cam_pts, fids, inliers):
    """ Update features """
    # Pre-check
    assert cam_idxs
    assert all(cam_idx in cam_imgs for cam_idx in cam_idxs)
    assert all(cam_idx in cam_pts for cam_idx in cam_idxs)

    # Filter outliers
    fids_in = list(fids)
    cam_kps, fids = self._filter_outliers(cam_idxs, cam_pts, inliers, 0, fids)

    # Update camera data
    for idx in cam_idxs:
      img = cam_imgs[idx]
      kps = cam_kps[idx]
      self.cam_data[idx] = FeatureTrackingData(idx, img, kps, None, fids)

    # Update lost features
    fids_out = set(fids)
    fids_lost = [x for x in fids_in if x not in fids_out]
    for fid in fids_lost:
      # feature overlaps
      if fid in self.feature_overlaps:
        self.feature_overlaps[fid] -= 1
        if self.feature_overlaps[fid] == 0:
          del self.feature_overlaps[fid]

  def _detect(self, image, prev_kps=None):
    """ Detect """
    assert image is not None
    kwargs = {'prev_kps': prev_kps, 'optflow_mode': True}
    kps = grid_detect(self.detector, image, **kwargs)
    return kps

  def _detect_overlaps(self, camera_images):
    """ Detect overlapping features """
    # Loop through camera overlaps
    for idx_i, idx_j in self.cam_overlaps:
      # Detect keypoints observed from cam_i
      img_i = camera_images[idx_i]
      img_j = camera_images[idx_j]
      prev_kps = self._get_keypoints(idx_i)
      kps_i = self._detect(img_i, prev_kps=prev_kps)
      if len(kps_i) < 10:
        continue

      # Track keypoint from cam_i to cam_j using optical flow
      kp_size = kps_i[0].size
      pts_i = np.array([kp.pt for kp in kps_i], dtype=np.float32)
      (pts_i, pts_j, optflow_inliers) = optflow_track(img_i, img_j, pts_i)

      # RANSAC
      cam_i = self.cam_params[idx_i]
      cam_j = self.cam_params[idx_j]
      ransac_inliers = ransac(pts_i, pts_j, cam_i, cam_j)

      # Reject outliers based on reprojection error
      reproj_inliers = self._reproj_filter(idx_i, idx_j, pts_i, pts_j)

      # Add features
      indices = [idx_i, idx_j]
      imgs = {idx_i: img_i, idx_j: img_j}
      pts = {idx_i: pts_i, idx_j: pts_j}
      inliers = optflow_inliers & ransac_inliers & reproj_inliers
      self._add_features(indices, imgs, pts, kp_size, inliers)

  def _detect_nonoverlaps(self, camera_images):
    for idx in self.cam_params:
      # Detect keypoints
      img = camera_images[idx]
      prev_kps = self._get_keypoints(idx)
      kps = self._detect(img, prev_kps=prev_kps)

      # Add features
      if not kps:
        return
      kp_size = kps[0].size
      self._add_features([idx], {idx: img}, {idx: kps}, kp_size)

  def _detect_new(self, camera_images):
    """ Detect new features """

    # Detect new features
    if self.mode == "TRACK_DEFAULT":
      self._detect_overlaps(camera_images)
      self._detect_nonoverlaps(camera_images)
    elif self.mode == "TRACK_OVERLAPS":
      self._detect_overlaps(camera_images)
    elif self.mode == "TRACK_INDEPENDENT":
      self._detect_nonoverlaps(camera_images)
    else:
      raise RuntimeError("Invalid FeatureTracker mode [%s]!" % self.mode)

  def _track_through_time(self, camera_images, cam_idx):
    """ Track features through time """

    # Setup images
    img_km1 = self.prev_camera_images[cam_idx]
    img_k = camera_images[cam_idx]

    # Setup keypoints and feature_ids
    kps_km1 = self._get_keypoints(cam_idx)
    feature_ids = self._get_feature_ids(cam_idx)
    pts_km1 = np.array([kp.pt for kp in kps_km1], dtype=np.float32)

    # Optical flow
    (pts_km1, pts_k, optflow_inliers) = optflow_track(img_km1, img_k, pts_km1)

    # RANSAC
    cam = self.cam_params[cam_idx]
    ransac_inliers = ransac(pts_km1, pts_k, cam, cam)

    # Form inliers list
    optflow_inliers = np.array(optflow_inliers)
    ransac_inliers = np.array(ransac_inliers)
    inliers = optflow_inliers & ransac_inliers

    return (pts_km1, pts_k, feature_ids, inliers)

  def _track_stereo(self, camera_images, idx_i, idx_j, pts_i):
    """ Track feature through stereo-pair """
    # Optical flow
    img_i = camera_images[idx_i]
    img_j = camera_images[idx_j]
    (pts_i, pts_j, optflow_inliers) = optflow_track(img_i, img_j, pts_i)

    # RANSAC
    cam_i = self.cam_params[idx_i]
    cam_j = self.cam_params[idx_j]
    ransac_inliers = ransac(pts_i, pts_j, cam_i, cam_j)

    # Reject outliers based on reprojection error
    reproj_inliers = self._reproj_filter(idx_i, idx_j, pts_i, pts_j)

    # Logical AND optflow_inliers and reproj_inliers
    ransac_inliers = np.array(ransac_inliers)
    optflow_inliers = np.array(optflow_inliers)
    reproj_inliers = np.array(reproj_inliers)
    inliers = optflow_inliers & ransac_inliers & reproj_inliers

    return (pts_i, pts_j, inliers)

  def _track_features(self, camera_images):
    """ Track features """
    # Track features in each camera
    for idx in self.cam_idxs:
      # Track through time
      track_results = self._track_through_time(camera_images, idx)
      (_, pts_k, feature_ids, inliers) = track_results

      # Update features
      idxs = [idx]
      imgs = {idx: camera_images[idx]}
      pts = {idx: pts_k}
      self._update_features(idxs, imgs, pts, feature_ids, inliers)

  def update(self, ts, camera_images):
    """ Update Feature Tracker """
    # Track features
    if self.frame_idx == 0:
      self._detect_new(camera_images)
      self.features_tracking = self.num_tracking()

    else:
      self._track_features(camera_images)
      if (self.num_tracking() / self.features_tracking) < 0.7:
        self._detect_new(camera_images)

    # Update
    self.frame_idx += 1
    self.prev_ts = ts
    self.prev_camera_images = camera_images

    return self.cam_data


def visualize_tracking(ft_data):
  """ Visualize feature tracking data """
  viz = []

  radius = 4
  green = (0, 255, 0)
  yellow = (0, 255, 255)
  thickness = 1
  linetype = cv2.LINE_AA

  # Find overlaps
  fids = {}
  feature_overlaps = set()
  for _, cam_data in ft_data.items():
    for n, _ in enumerate(cam_data.keypoints):
      fid = cam_data.feature_ids[n]
      fids[fid] = (fids[fid] + 1) if fid in fids else 1

      if fids[fid] > 1:
        feature_overlaps.add(fid)

  # Draw features being tracked in each camera
  for _, cam_data in ft_data.items():
    img = cam_data.image
    cam_viz = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for n, kp in enumerate(cam_data.keypoints):
      fid = cam_data.feature_ids[n]
      color = green if fid in feature_overlaps else yellow
      p = (int(kp.pt[0]), int(kp.pt[1])) if hasattr(kp, 'pt') else kp
      cv2.circle(cam_viz, p, radius, color, thickness, lineType=linetype)

    viz.append(cam_viz)

  return cv2.hconcat(viz)


# STATE-ESTIMATOR #############################################################


class KeyFrame:
  """ Key Frame """

  def __init__(self, ts, images, data):
    self.ts = ts
    self.images = images
    self.data = data


class Tracker:
  """ Tracker """

  def __init__(self):
    self.feature_tracker = FeatureTracker()
    self.keyframes = []

    self.cam_params = {}
    self.cam_exts = {}

  def add_camera(self, cam_idx, cam_params, cam_exts):
    """ Add camera """
    self.cam_params[cam_idx] = cam_params
    self.cam_exts[cam_idx] = cam_exts
    self.feature_tracker.add_camera(cam_idx, cam_params, cam_exts)

  def add_overlap(self, idx_i, idx_j):
    """ Add overlap """
    self.feature_tracker.add_overlap(idx_i, idx_j)

  def vision_callback(self, ts, images):
    """ Vision callback """
    ft_data = self.feature_tracker.update(ts, images)
    self.keyframes.append(KeyFrame(ts, images, ft_data))

    return ft_data


###############################################################################
# CALIBRATION
###############################################################################


@dataclass
class CalibTarget:
  """ Calibration Target """
  nb_rows: int
  nb_cols: int
  tag_size: float
  tag_spacing: float


def calib_generate_poses(calib_target):
  """ Generate calibration poses infront of the calibration target """
  # Settings
  calib_width = (calib_target.nb_cols - 1.0) * calib_target.tag_size
  calib_height = (calib_target.nb_rows - 1.0) * calib_target.tag_size
  calib_center = np.array([calib_width / 2.0, calib_height / 2.0, 0.0])

  # Pose settings
  x_range = np.linspace(-0.3, 0.3, 5)
  y_range = np.linspace(-0.3, 0.3, 5)
  z_range = np.linspace(0.2, 0.5, 5)

  # Generate camera positions infront of the AprilGrid target r_TC
  cam_pos = zeros((3, len(x_range) * len(y_range) * len(z_range)))
  pos_idx = 1
  for x in x_range:
    for y in y_range:
      for z in z_range:
        r_TC = np.array([x, y, z]) + calib_center  # Calib center as offset
        cam_pos[:, pos_idx] = r_TC
        pos_idx += 1

  # For each position create a camera pose that "looks at" the AprilGrid
  # center in the target frame, T_TC.
  poses = []
  for i in range(cam_pos.shape[1]):
    T_TC = lookat(cam_pos[:, i], calib_center)
    poses.append(T_TC)


def calib_generate_random_poses(calib_target, nb_poses):
  """ Generate random calibration poses infront of the calibration target """
  # Settings
  calib_width = (calib_target.nb_cols - 1.0) * calib_target.tag_size
  calib_height = (calib_target.nb_rows - 1.0) * calib_target.tag_size
  calib_center = np.array([calib_width / 2.0, calib_height / 2.0, 0.0])

  att_range = [deg2rad(-20.0), deg2rad(20.0)]
  x_range = [-0.5, 0.5]
  y_range = [-0.5, 0.5]
  z_range = [0.5, 0.7]

  # For each position create a camera pose that "looks at" the AprilGrid
  # center in the target frame, T_TC.
  poses = []
  for _ in range(nb_poses):
    # Generate random pose
    x = np.random.uniform(x_range[0], x_range[1])
    y = np.random.uniform(y_range[0], y_range[1])
    z = np.random.uniform(z_range[0], z_range[1])
    r_TC = calib_center + np.array([x, y, z])
    T_TC = lookat(r_TC, calib_center)

    # Perturb the pose a little so it doesn't look at the center directly
    yaw = np.random.uniform(att_range)
    pitch = np.random.uniform(att_range)
    roll = np.random.uniform(att_range)
    C_perturb = euler321(yaw, pitch, roll)
    r_perturb = zeros((3, 1))
    T_perturb = tf(C_perturb, r_perturb)
    poses.append(T_TC * T_perturb)

  return poses


@dataclass
class AprilGrid:
  """ AprilGrid """
  tag_rows: int = 6
  tag_cols: int = 6
  tag_sizse: float = 0.088
  tag_spacing: float = 0.3
  keypoints: List = field(default_factory=[])
  object_points: List = field(default_factory=[])

  def __post_init__(self):
    # Form object points
    self.object_points = []
    nb_tags = self.tag_rows * self.tag_cols
    for tag_id in range(nb_tags - 1):
      # Calculate the AprilGrid index using tag id
      [i, j] = self.grid_index(tag_id)

      # Calculate the x and y of the tag origin (bottom left corner of tag)
      # relative to grid origin (bottom left corner of entire grid)
      x = j * (self.tag_sizse + self.tag_sizse * self.tag_spacing)
      y = i * (self.tag_sizse + self.tag_sizse * self.tag_spacing)

      # Bottom left
      pt_bl = [x, y, 0]
      # Bottom right
      pt_br = [x + self.tag_sizse, y, 0]
      # Top right
      pt_tr = [x + self.tag_sizse, y + self.tag_sizse, 0]
      # Top left
      pt_tl = [x, y + self.tag_sizse, 0]

      # Tag object points
      tag_points = [pt_bl, pt_br, pt_tr, pt_tl]

      # Add to total object points
      self.object_points.append(tag_points)

  def grid_index(self, tag_id):
    """ Calculate grid index from tag id """
    assert tag_id < (self.tag_rows * self.tag_cols) and id >= 0
    i = floor(tag_id / self.tag_cols)
    j = floor(tag_id % self.tag_cols)
    return (i, j)


################################################################################
# DATASET
################################################################################

# TIMELINE######################################################################


@dataclass
class CameraEvent:
  """ Camera Event """
  ts: int
  cam_idx: int
  feature_ids: List[int]
  measurements: List[np.array]


@dataclass
class CameraImageEvent:
  """ Camera Event """
  ts: int
  cam_idx: int
  image_path: str


@dataclass
class ImuEvent:
  """ IMU Event """
  ts: int
  imu_idx: int
  acc: np.array
  gyr: np.array


@dataclass
class Timeline:
  """ Timeline """

  def __init__(self):
    self.data = {}

  def num_timestamps(self):
    """ Return number of timestamps """
    return len(self.data)

  def num_events(self):
    """ Return number of events """
    nb_events = 0
    for _, events in self.data:
      nb_events += len(events)
    return nb_events

  def get_timestamps(self):
    """ Get timestamps """
    return self.data.keys()

  def add_event(self, ts, event):
    """ Add event """
    if ts not in self.data:
      self.data[ts] = [event]
    else:
      self.data[ts].append(event)

  def get_events(self, ts):
    """ Get events """
    return self.data[ts]


# EUROC ########################################################################


class EurocSensor:
  """ Euroc Sensor """

  def __init__(self, yaml_path):
    # Load yaml file
    config = load_yaml(yaml_path)

    # General sensor definitions.
    self.sensor_type = config.sensor_type
    self.comment = config.comment

    # Sensor extrinsics wrt. the body-frame.
    self.T_BS = np.array(config.T_BS.data).reshape((4, 4))

    # Camera specific definitions.
    if config.sensor_type == "camera":
      self.rate_hz = config.rate_hz
      self.resolution = config.resolution
      self.camera_model = config.camera_model
      self.intrinsics = config.intrinsics
      self.distortion_model = config.distortion_model
      self.distortion_coefficients = config.distortion_coefficients

    elif config.sensor_type == "imu":
      self.rate_hz = config.rate_hz
      self.gyro_noise_density = config.gyroscope_noise_density
      self.gyro_random_walk = config.gyroscope_random_walk
      self.accel_noise_density = config.accelerometer_noise_density
      self.accel_random_walk = config.accelerometer_random_walk


@dataclass
class EurocImuData:
  """ Euroc Imu data """
  timestamps: List[float]
  acc: Dict[float, np.array]
  gyr: Dict[float, np.array]


class EurocDataset:
  """ Euroc Dataset """

  def __init__(self, data_path):
    # Form sensor paths
    self.imu0_path = os.path.join(data_path, 'mav0', 'imu0')
    self.cam0_path = os.path.join(data_path, 'mav0', 'cam0')
    self.cam1_path = os.path.join(data_path, 'mav0', 'cam1')

    # Data
    self.timestamps = []
    # -- Configs
    self.imu0_config = {}
    self.cam0_config = {}
    self.cam1_config = {}
    # -- Measurements
    self.imu0_data = {}
    self.cam0_images = {}
    self.cam1_images = {}
    # -- Timeline
    self.timeline = Timeline()

    # Load
    self._load_euroc_dataset(data_path)

  def _load_imu_data(self):
    # Load config
    config = EurocSensor(os.path.join(self.imu0_path, 'sensor.yaml'))

    # Load data
    imu_data = pandas.read_csv(os.path.join(self.imu0_path, 'data.csv'))
    timestamps = imu_data['#timestamp [ns]']
    gyr_x = imu_data['w_RS_S_x [rad s^-1]']
    gyr_y = imu_data['w_RS_S_y [rad s^-1]']
    gyr_z = imu_data['w_RS_S_z [rad s^-1]']
    acc_x = imu_data['a_RS_S_x [m s^-2]']
    acc_y = imu_data['a_RS_S_y [m s^-2]']
    acc_z = imu_data['a_RS_S_z [m s^-2]']

    gyr_data = {}
    acc_data = {}
    idx = 0
    for ts in timestamps:
      acc_data[ts] = np.array([acc_x[idx], acc_y[idx], acc_z[idx]])
      gyr_data[ts] = np.array([gyr_x[idx], gyr_y[idx], gyr_z[idx]])
      idx += 1

    data = EurocImuData(timestamps, acc_data, gyr_data)

    return (config, data)

  def _load_camera_data(self, cam_path):
    config = EurocSensor(os.path.join(cam_path, 'sensor.yaml'))
    image_paths = sorted(glob.glob(os.path.join(cam_path, 'data', '*.png')))
    images = {}

    for img_file in image_paths:
      ts_str, _ = os.path.basename(img_file).split('.')
      ts = int(ts_str)
      images[ts] = img_file
      self.timestamps.append(ts)

    return (config, images)

  def _load_euroc_dataset(self, data_path):
    # Check if directory exists
    if os.path.isdir(data_path) is False:
      raise RuntimeError(f"Path {data_path} does not exist!")

    # Load data
    self.imu0_config, self.imu0_data = self._load_imu_data()
    self.cam0_config, self.cam0_images = self._load_camera_data(self.cam0_path)
    self.cam1_config, self.cam1_images = self._load_camera_data(self.cam1_path)

    # Timestamps
    self.timestamps = sorted(list(set(self.timestamps)))

    # Form timeline
    # -- Add imu0 events
    imu_idx = 0
    for ts in self.imu0_data.timestamps:
      acc = self.imu0_data.acc[ts]
      gyr = self.imu0_data.gyr[ts]
      imu_event = ImuEvent(ts, imu_idx, acc, gyr)
      self.timeline.add_event(ts, imu_event)

    # -- Add cam0 events
    for ts, img_path in self.cam0_images.items():
      self.timeline.add_event(ts, CameraImageEvent(ts, 0, img_path))

    # -- Add cam1 events
    for ts, img_path in self.cam1_images.items():
      self.timeline.add_event(ts, CameraImageEvent(ts, 1, img_path))


###############################################################################
# SIMULATION
###############################################################################

# UTILS #######################################################################


def create_3d_features(x_bounds, y_bounds, z_bounds, nb_features):
  """ Create 3D features randomly """
  features = zeros((nb_features, 3))
  for i in range(nb_features):
    features[i, 0] = random.uniform(*x_bounds)
    features[i, 1] = random.uniform(*y_bounds)
    features[i, 2] = random.uniform(*z_bounds)
  return features


def create_3d_features_perimeter(origin, dim, nb_features):
  """ Create 3D features in a square """
  assert len(origin) == 3
  assert len(dim) == 3
  assert nb_features > 0

  # Dimension of the outskirt
  w, l, h = dim

  # Features per side
  nb_fps = int(nb_features / 4.0)

  # Features in the east side
  x_bounds = [origin[0] - w, origin[0] + w]
  y_bounds = [origin[1] + l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  east = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the north side
  x_bounds = [origin[0] + w, origin[0] + w]
  y_bounds = [origin[1] - l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  north = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the west side
  x_bounds = [origin[0] - w, origin[0] + w]
  y_bounds = [origin[1] - l, origin[1] - l]
  z_bounds = [origin[2] - h, origin[2] + h]
  west = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the south side
  x_bounds = [origin[0] - w, origin[0] - w]
  y_bounds = [origin[1] - l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  south = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Stack features and return
  return np.block([[east], [north], [west], [south]])


# SIMULATION ##################################################################


class SimCameraFrame:
  """ Sim camera frame """

  def __init__(self, ts, cam_idx, camera, T_WCi, points_W):
    assert T_WCi.shape == (4, 4)
    assert points_W.shape[0] > 0
    assert points_W.shape[1] == 3

    self.ts = ts
    self.cam_idx = cam_idx
    self.feature_ids = []
    self.measurements = []

    # Setup
    T_CiW = tf_inv(T_WCi)
    cam_geom = camera.data
    cam_params = camera.param
    img_w, img_h = cam_geom.resolution
    nb_points = points_W.shape[0]

    # Simulate camera frame
    self.measurements = []
    self.feature_ids = []

    for i in range(nb_points):
      # Project point from world frame to camera frame
      p_W = points_W[i, :]
      p_C = tf_point(T_CiW, p_W)
      z = cam_geom.project(cam_params, p_C)

      # Check to see if image point is within image plane
      x_ok = (z[0] < img_w) and (z[0] > 0.0)
      y_ok = (z[1] < img_h) and (z[1] > 0.0)
      if x_ok and y_ok:
        self.measurements.append(z)
        self.feature_ids.append(i)


class SimCameraData:
  """ Sim camera data """

  def __init__(self, cam_idx, camera, features):
    self.cam_idx = cam_idx
    self.camera = camera
    self.features = features
    self.timestamps = []
    self.poses = {}
    self.frames = {}


class SimImuData:
  """ Sim imu data """

  def __init__(self, imu_idx):
    self.imu_idx = imu_idx
    self.timestamps = []
    self.poses = {}
    self.vel = {}
    self.acc = {}
    self.gyr = {}

  def form_imu_buffer(self, start_idx, end_idx):
    """ Form ImuBuffer """
    imu_ts = self.timestamps[start_idx:end_idx]
    imu_acc = []
    imu_gyr = []
    for ts in self.timestamps:
      imu_acc.append(self.acc[ts])
      imu_gyr.append(self.gyr[ts])

    return ImuBuffer(imu_ts, imu_acc, imu_gyr)


class SimData:
  """ Sim data """

  def __init__(self, circle_r, circle_v, **kwargs):
    # Settings
    self.circle_r = circle_r
    self.circle_v = circle_v
    self.cam_rate = 10.0
    self.imu_rate = 200.0
    self.nb_features = 200

    # Trajectory data
    self.g = np.array([0.0, 0.0, 9.81])
    self.circle_dist = 2.0 * pi * circle_r
    self.time_taken = self.circle_dist / self.circle_v
    self.w = -2.0 * pi * (1.0 / self.time_taken)
    self.theta_init = pi
    self.yaw_init = pi / 2.0
    self.features = self._setup_features()

    # Simulate IMU
    self.imu0_data = None
    if kwargs.get("sim_imu", True):
      self.imu0_data = self._sim_imu(0)

    # Simulate camera
    self.mcam_data = {}
    if kwargs.get("sim_cams", True):
      # -- cam0
      self.cam0_params = self._setup_camera(0)
      C_BC0 = euler321(*deg2rad([-90.0, 0.0, -90.0]))
      r_BC0 = np.array([0.0, 0.0, 0.0])
      self.T_BC0 = tf(C_BC0, r_BC0)
      self.mcam_data[0] = self._sim_cam(0, self.cam0_params, self.T_BC0)
      # -- cam1
      self.cam1_params = self._setup_camera(1)
      C_BC1 = euler321(*deg2rad([-90.0, 0.0, -90.0]))
      r_BC1 = np.array([0.0, 0.0, 0.0])
      self.T_BC1 = tf(C_BC1, r_BC1)
      # -- Multicam data
      self.mcam_data[1] = self._sim_cam(1, self.cam1_params, self.T_BC1)

    # Timeline
    self.timeline = self._form_timeline()

  @staticmethod
  def _setup_camera(cam_idx):
    """ Setup camera """
    res = [640, 480]
    fov = 90.0
    fx = focal_length(res[0], fov)
    fy = focal_length(res[0], fov)
    cx = res[0] / 2.0
    cy = res[0] / 2.0

    proj_model = "pinhole"
    dist_model = "radtan4"
    proj_params = [fx, fy, cx, cy]
    dist_params = [-0.01, 0.01, 1e-4, 1e-4]
    params = np.block([*proj_params, *dist_params])

    return camera_params_setup(cam_idx, res, proj_model, dist_model, params)

  def _setup_features(self):
    """ Setup features """
    origin = [0, 0, 0]
    dim = [self.circle_r * 2.0, self.circle_r * 2.0, self.circle_r * 1.5]
    return create_3d_features_perimeter(origin, dim, self.nb_features)

  def _sim_imu(self, imu_idx):
    """ Simulate IMU """
    sim_data = SimImuData(imu_idx)

    time = 0.0
    dt = 1.0 / self.imu_rate
    theta = self.theta_init
    yaw = self.yaw_init

    while time <= self.time_taken:
      # Timestamp
      ts = sec2ts(time)

      # IMU pose
      rx = self.circle_r * cos(theta)
      ry = self.circle_r * sin(theta)
      rz = 0.0
      r_WS = np.array([rx, ry, rz])
      C_WS = euler321(yaw, 0.0, 0.0)
      T_WS = tf(C_WS, r_WS)

      # IMU velocity
      vx = -self.circle_r * self.w * sin(theta)
      vy = self.circle_r * self.w * cos(theta)
      vz = 0.0
      v_WS = np.array([vx, vy, vz])

      # IMU acceleration
      ax = -self.circle_r * self.w**2 * cos(theta)
      ay = -self.circle_r * self.w**2 * sin(theta)
      az = 0.0
      a_WS = np.array([ax, ay, az])

      # IMU angular velocity
      wx = 0.0
      wy = 0.0
      wz = self.w
      w_WS = np.array([wx, wy, wz])

      # IMU measurements
      acc = C_WS.T @ (a_WS + self.g)
      gyr = C_WS.T @ w_WS

      # Update
      sim_data.timestamps.append(ts)
      sim_data.poses[ts] = T_WS
      sim_data.vel[ts] = v_WS
      sim_data.acc[ts] = acc
      sim_data.gyr[ts] = gyr

      theta += self.w * dt
      yaw += self.w * dt
      time += dt

    return sim_data

  def _sim_cam(self, cam_idx, cam_params, T_BCi):
    """ Simulate camera """
    sim_data = SimCameraData(cam_idx, cam_params, self.features)

    time = 0.0
    dt = 1.0 / self.cam_rate
    theta = self.theta_init
    yaw = self.yaw_init

    while time <= self.time_taken:
      # Timestamp
      ts = sec2ts(time)

      # Body pose
      rx = self.circle_r * cos(theta)
      ry = self.circle_r * sin(theta)
      rz = 0.0
      r_WB = [rx, ry, rz]
      C_WB = euler321(yaw, 0.0, 0.0)
      T_WB = tf(C_WB, r_WB)

      # Simulate camera pose and camera frame
      T_WCi = T_WB @ T_BCi
      cam_frame = SimCameraFrame(ts, cam_idx, cam_params, T_WCi, self.features)
      sim_data.timestamps.append(ts)
      sim_data.poses[ts] = T_WCi
      sim_data.frames[ts] = cam_frame

      # Update
      theta += self.w * dt
      yaw += self.w * dt
      time += dt

    return sim_data

  def _form_timeline(self):
    """ Form timeline """
    # Form timeline
    timeline = Timeline()

    # -- Add imu events
    imu_idx = self.imu0_data.imu_idx
    for ts in self.imu0_data.timestamps:
      acc = self.imu0_data.acc[ts]
      gyr = self.imu0_data.gyr[ts]
      imu_event = ImuEvent(ts, imu_idx, acc, gyr)
      timeline.add_event(ts, imu_event)

    # -- Add camera events
    for cam_idx, cam_data in self.mcam_data.items():
      for ts in cam_data.timestamps:
        frame = cam_data.frames[ts]
        fids = frame.feature_ids
        kps = frame.measurements
        cam_event = CameraEvent(ts, cam_idx, fids, kps)
        timeline.add_event(ts, cam_event)

    return timeline

  @staticmethod
  def create_or_load(circle_r, circle_v, pickle_path):
    """ Create or load SimData """
    sim_data = None

    if os.path.exists(pickle_path):
      with open(pickle_path, 'rb') as f:
        sim_data = pickle.load(f)
    else:
      sim_data = SimData(circle_r, circle_v)
      with open(pickle_path, 'wb') as f:
        pickle.dump(sim_data, f)
        f.flush()

    return sim_data


###############################################################################
#                               UNITTESTS
###############################################################################

import unittest

euroc_data_path = '/data/euroc/raw/V1_01'

# euroc_data_path = '/data/euroc/raw/MH_01'

# LINEAR ALGEBRA ##############################################################


class TestLinearAlgebra(unittest.TestCase):
  """ Test Linear Algebra """

  def test_normalize(self):
    """ Test normalize() """
    x = np.array([1.0, 2.0, 3.0])
    x_prime = normalize(x)
    self.assertTrue(isclose(norm(x_prime), 1.0))

  def test_skew(self):
    """ Test skew() """
    x = np.array([1.0, 2.0, 3.0])
    S = np.array([[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]])
    self.assertTrue(matrix_equal(S, skew(x)))

  def test_skew_inv(self):
    """ Test skew_inv() """
    x = np.array([1.0, 2.0, 3.0])
    S = np.array([[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]])
    self.assertTrue(matrix_equal(x, skew_inv(S)))

  def test_matrix_equal(self):
    """ Test matrix_equal() """
    A = ones((3, 3))
    B = ones((3, 3))
    self.assertTrue(matrix_equal(A, B))

    C = 2.0 * ones((3, 3))
    self.assertFalse(matrix_equal(A, C))

  # def test_check_jacobian(self):
  #   step_size = 1e-6
  #   threshold = 1e-5
  #
  #   x = 2
  #   y0 = x**2
  #   y1 = (x + step_size)**2
  #   jac = 2 * x
  #   fdiff = y1 - y0
  #
  #   jac_name = "jac"
  #   fdiff = (y1 - y0) / step_size
  #   self.assertTrue(check_jacobian(jac_name, fdiff, jac, threshold))


class TestLie(unittest.TestCase):
  """ Test Lie algebra functions """

  def test_Exp_Log(self):
    """ Test Exp() and Log() """
    pass


# TRANSFORM ###################################################################


class TestTransform(unittest.TestCase):
  """ Test transform functions """

  def test_homogeneous(self):
    """ Test homogeneous() """
    p = np.array([1.0, 2.0, 3.0])
    hp = homogeneous(p)
    self.assertTrue(hp[0] == 1.0)
    self.assertTrue(hp[1] == 2.0)
    self.assertTrue(hp[2] == 3.0)
    self.assertTrue(len(hp) == 4)

  def test_dehomogeneous(self):
    """ Test dehomogeneous() """
    p = np.array([1.0, 2.0, 3.0])
    hp = np.array([1.0, 2.0, 3.0, 1.0])
    p = dehomogeneous(hp)
    self.assertTrue(p[0] == 1.0)
    self.assertTrue(p[1] == 2.0)
    self.assertTrue(p[2] == 3.0)
    self.assertTrue(len(p) == 3)

  def test_rotx(self):
    """ Test rotx() """
    x = np.array([0.0, 1.0, 0.0])
    C = rotx(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 0.0, 1.0]))

  def test_roty(self):
    """ Test roty() """
    x = np.array([1.0, 0.0, 0.0])
    C = roty(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 0.0, -1.0]))

  def test_rotz(self):
    """ Test rotz() """
    x = np.array([1.0, 0.0, 0.0])
    C = rotz(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 1.0, 0.0]))

  def test_aa2quat(self):
    """ Test aa2quat() """
    pass

  def test_rvec2rot(self):
    """ Test rvec2quat() """
    pass

  def test_vecs2axisangle(self):
    """ Test vecs2axisangle() """
    pass

  def test_euler321(self):
    """ Test euler321() """
    C = euler321(0.0, 0.0, 0.0)
    self.assertTrue(np.array_equal(C, eye(3)))

  def test_euler2quat_and_quat2euler(self):
    """ Test euler2quat() and quat2euler() """
    y_in = deg2rad(3.0)
    p_in = deg2rad(2.0)
    r_in = deg2rad(1.0)

    q = euler2quat(y_in, p_in, r_in)
    ypr_out = quat2euler(q)

    self.assertTrue(len(q) == 4)
    self.assertTrue(abs(y_in - ypr_out[0]) < 1e-5)
    self.assertTrue(abs(p_in - ypr_out[1]) < 1e-5)
    self.assertTrue(abs(r_in - ypr_out[2]) < 1e-5)

  def test_quat2rot(self):
    """ Test quat2rot() """
    pass

  def test_rot2euler(self):
    """ Test rot2euler() """
    pass

  def test_rot2quat(self):
    """ Test rot2quat() """
    pass

  def test_quat_norm(self):
    """ Test quat_norm() """
    q = np.array([1.0, 0.0, 0.0, 0.0])
    self.assertTrue(isclose(quat_norm(q), 1.0))

  def test_quat_normalize(self):
    """ Test quat_normalize() """
    q = np.array([1.0, 0.1, 0.2, 0.3])
    q = quat_normalize(q)
    self.assertTrue(isclose(quat_norm(q), 1.0))

  def test_quat_conj(self):
    """ Test quat_conj() """
    pass

  def test_quat_inv(self):
    """ Test quat_inv() """
    pass

  def test_quat_mul(self):
    """ Test quat_mul() """
    p = euler2quat(deg2rad(3.0), deg2rad(2.0), deg2rad(1.0))
    q = euler2quat(deg2rad(1.0), deg2rad(2.0), deg2rad(3.0))
    r = quat_mul(p, q)
    self.assertTrue(r is not None)

  def test_quat_omega(self):
    """ Test quat_omega() """
    pass

  def test_tf(self):
    """ Test tf() """
    pass
    # pose = [1.0; 0.0; 0.0; 0.0; 1.0; 2.0; 3.0];
    # T = tf(pose);
    # self.assertTrue(isequal(T(0:3, 0:3), eye(3)) == 1);
    # self.assertTrue(isequal(T(0:3, 4), [1; 2; 3]) == 1);

    # C = [[1.0, 0.0, 0.0];
    #     [0.0, 2.0, 0.0];
    #     [0.0, 0.0, 3.0]];
    # r = [1.0; 2.0; 3.0];
    # T = tf(C, r);
    # self.assertTrue(isequal(T(0:3, 0:3), C) == 1);
    # self.assertTrue(isequal(T(0:3, 4), r) == 1);


# CV ##########################################################################


class TestCV(unittest.TestCase):
  """ Test computer vision functions """

  def setUp(self):
    # Camera
    img_w = 640
    img_h = 480
    fx = focal_length(img_w, 90.0)
    fy = focal_length(img_w, 90.0)
    cx = img_w / 2.0
    cy = img_h / 2.0
    self.proj_params = [fx, fy, cx, cy]

    # Camera pose in world frame
    C_WC = euler321(-pi / 2, 0.0, -pi / 2)
    r_WC = np.array([0.0, 0.0, 0.0])
    self.T_WC = tf(C_WC, r_WC)

    # 3D World point
    self.p_W = np.array([10.0, 0.0, 0.0])

    # Point w.r.t camera
    self.p_C = tf_point(inv(self.T_WC), self.p_W)
    self.x = np.array([self.p_C[0] / self.p_C[2], self.p_C[1] / self.p_C[2]])

  def test_linear_triangulation(self):
    """ Test linear_triangulation() """
    # Camera i - Camera j extrinsics
    C_CiCj = eye(3)
    r_CiCj = np.array([0.05, 0.0, 0.0])
    T_CiCj = tf(C_CiCj, r_CiCj)

    # Camera 0 pose in world frame
    C_WCi = euler321(-pi / 2, 0.0, -pi / 2)
    r_WCi = np.array([0.0, 0.0, 0.0])
    T_WCi = tf(C_WCi, r_WCi)

    # Camera 1 pose in world frame
    T_WCj = T_WCi @ T_CiCj

    # Projection matrices P_i and P_j
    P_i = pinhole_P(self.proj_params, eye(4))
    P_j = pinhole_P(self.proj_params, T_CiCj)

    # Test multiple times
    nb_tests = 100
    for _ in range(nb_tests):
      # Project feature point p_W to image plane
      x = np.random.uniform(-0.05, 0.05)
      y = np.random.uniform(-0.05, 0.05)
      p_W = np.array([10.0, x, y])
      p_Ci_gnd = tf_point(inv(T_WCi), p_W)
      p_Cj_gnd = tf_point(inv(T_WCj), p_W)
      z_i = pinhole_project(self.proj_params, p_Ci_gnd)
      z_j = pinhole_project(self.proj_params, p_Cj_gnd)

      # Triangulate
      p_Ci_est = linear_triangulation(P_i, P_j, z_i, z_j)
      self.assertTrue(np.allclose(p_Ci_est, p_Ci_gnd))

  def test_pinhole_K(self):
    """ Test pinhole_K() """
    fx = 1.0
    fy = 2.0
    cx = 3.0
    cy = 4.0
    proj_params = [fx, fy, cx, cy]
    K = pinhole_K(proj_params)
    expected = np.array([[1.0, 0.0, 3.0], [0.0, 2.0, 4.0], [0.0, 0.0, 1.0]])

    self.assertTrue(np.array_equal(K, expected))

  def test_pinhole_project(self):
    """ Test pinhole_project() """
    z = pinhole_project(self.proj_params, self.p_C)
    self.assertTrue(isclose(z[0], 320.0))
    self.assertTrue(isclose(z[1], 240.0))

  def test_pinhole_params_jacobian(self):
    """ Test pinhole_params_jacobian() """
    # Pinhole params jacobian
    fx, fy, cx, cy = self.proj_params
    z = np.array([fx * self.x[0] + cx, fy * self.x[1] + cy])
    J = pinhole_params_jacobian(self.x)

    # Perform numerical diff to obtain finite difference
    step_size = 1e-6
    tol = 1e-4
    finite_diff = zeros((2, 4))

    for i in range(4):
      params_diff = list(self.proj_params)
      params_diff[i] += step_size
      fx, fy, cx, cy = params_diff

      z_diff = np.array([fx * self.x[0] + cx, fy * self.x[1] + cy])
      finite_diff[0:2, i] = (z_diff - z) / step_size

    self.assertTrue(matrix_equal(finite_diff, J, tol, True))

  def test_pinhole_point_jacobian(self):
    """ Test pinhole_point_jacobian() """
    # Pinhole params jacobian
    fx, fy, cx, cy = self.proj_params
    z = np.array([fx * self.x[0] + cx, fy * self.x[1] + cy])
    J = pinhole_point_jacobian(self.proj_params)

    # Perform numerical diff to obtain finite difference
    step_size = 1e-6
    tol = 1e-4
    finite_diff = zeros((2, 2))

    for i in range(2):
      x_diff = list(self.x)
      x_diff[i] += step_size

      z_diff = np.array([fx * x_diff[0] + cx, fy * x_diff[1] + cy])
      finite_diff[0:2, i] = (z_diff - z) / step_size

    self.assertTrue(matrix_equal(finite_diff, J, tol, True))


# STATE ESTIMATION ############################################################


class TestFactors(unittest.TestCase):
  """ Test factors """

  def test_pose_factor(self):
    """ Test pose factor """
    # Setup camera pose T_WC
    rot = euler2quat(-pi / 2.0, 0.0, -pi / 2.0)
    trans = np.array([0.1, 0.2, 0.3])
    T_WC = tf(rot, trans)

    rot = euler2quat(-pi / 2.0 + 0.01, 0.0 + 0.01, -pi / 2.0 + 0.01)
    trans = np.array([0.1 + 0.01, 0.2 + 0.01, 0.3 + 0.01])
    T_WC_diff = tf(rot, trans)
    pose_est = pose_setup(0, T_WC_diff)

    # Create factor
    param_ids = [0]
    pose_factor = pose_factor_setup(param_ids, T_WC)

    # Evaluate factor
    params = [pose_est]
    pose_factor.eval(params)

    # Test jacobians
    self.assertTrue(check_factor_jacobian(pose_factor, params, 0, "J_pose"))

  def test_ba_factor(self):
    """ Test ba factor """
    # Setup camera pose T_WC
    rot = euler2quat(-pi / 2.0, 0.0, -pi / 2.0)
    trans = np.array([0.1, 0.2, 0.3])
    T_WC = tf(rot, trans)
    cam_pose = pose_setup(0, T_WC)

    # Setup cam0
    cam_idx = 0
    img_w = 640
    img_h = 480
    res = [img_w, img_h]
    fov = 60.0
    fx = focal_length(img_w, fov)
    fy = focal_length(img_h, fov)
    cx = img_w / 2.0
    cy = img_h / 2.0
    params = [fx, fy, cx, cy, -0.01, 0.01, 1e-4, 1e-4]
    cam_params = camera_params_setup(cam_idx, res, "pinhole", "radtan4", params)
    cam_geom = camera_geometry_setup(cam_idx, res, "pinhole", "radtan4")

    # Setup feature
    p_W = np.array([10, random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)])
    # -- Feature XYZ parameterization
    feature = feature_setup(p_W)
    # # -- Feature inverse depth parameterization
    # param = idp_param(camera, T_WC, z)
    # feature = feature_init(0, param)
    # -- Calculate image point
    p_C = tf_point(inv(T_WC), p_W)
    z = cam_geom.project(cam_params.param, p_C)

    # Setup factor
    param_ids = [0, 1, 2]
    ba_factor = ba_factor_setup(param_ids, z, cam_geom)

    # Evaluate factor
    params = [cam_pose, feature, cam_params]
    ba_factor.eval(params)

    # Test jacobians
    self.assertTrue(check_factor_jacobian(ba_factor, params, 0, "J_cam_pose"))
    self.assertTrue(check_factor_jacobian(ba_factor, params, 1, "J_feature"))
    self.assertTrue(check_factor_jacobian(ba_factor, params, 2, "J_cam_params"))

  def test_vision_factor(self):
    """ Test vision factor """
    # Setup camera extrinsics T_BCi
    rot = euler2quat(0.01, 0.01, 0.03)
    trans = np.array([0.001, 0.002, 0.003])
    T_WB = tf(rot, trans)
    pose = pose_setup(0, T_WB)

    # Setup camera extrinsics T_BCi
    rot = euler2quat(-pi / 2.0, 0.0, -pi / 2.0)
    trans = np.array([0.1, 0.2, 0.3])
    T_BCi = tf(rot, trans)
    cam_exts = extrinsics_setup(T_BCi)

    # Setup cam0
    cam_idx = 0
    img_w = 640
    img_h = 480
    res = [img_w, img_h]
    fov = 60.0
    fx = focal_length(img_w, fov)
    fy = focal_length(img_h, fov)
    cx = img_w / 2.0
    cy = img_h / 2.0
    params = [fx, fy, cx, cy, -0.01, 0.01, 1e-4, 1e-4]
    cam_params = camera_params_setup(cam_idx, res, "pinhole", "radtan4", params)
    cam_geom = camera_geometry_setup(cam_idx, res, "pinhole", "radtan4")

    # Setup feature
    p_W = np.array([10, random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)])
    # -- Feature XYZ parameterization
    feature = feature_setup(p_W)
    # # -- Feature inverse depth parameterization
    # param = idp_param(camera, T_WC, z)
    # feature = feature_init(0, param)
    # -- Calculate image point
    T_WCi = T_WB * T_BCi
    p_C = tf_point(inv(T_WCi), p_W)
    z = cam_geom.project(cam_params.param, p_C)

    # Setup factor
    param_ids = [0, 1, 2, 3]
    factor = vision_factor_setup(param_ids, z, cam_geom)

    # Evaluate factor
    params = [pose, cam_exts, feature, cam_params]
    # r, jacs = factor.eval(params)
    factor.eval(params)

    # Test jacobians
    self.assertTrue(check_factor_jacobian(factor, params, 0, "J_pose"))
    self.assertTrue(check_factor_jacobian(factor, params, 1, "J_cam_exts"))
    self.assertTrue(check_factor_jacobian(factor, params, 2, "J_feature"))
    self.assertTrue(check_factor_jacobian(factor, params, 3, "J_cam_params"))

  def test_imu_factor_propagate(self):
    """ Test IMU factor propagate """
    # Sim imu data
    circle_r = 1.0
    circle_v = 1.0
    sim_data = SimData(circle_r, circle_v, sim_cams=False)
    imu_data = sim_data.imu0_data

    # Setup imu parameters
    noise_acc = 0.08  # accelerometer measurement noise stddev.
    noise_gyr = 0.004  # gyroscope measurement noise stddev.
    noise_ba = 0.00004  # accelerometer bias random work noise stddev.
    noise_bg = 2.0e-6  # gyroscope bias random work noise stddev.
    imu_params = ImuParams(noise_acc, noise_gyr, noise_ba, noise_bg)

    # Setup imu buffer
    start_idx = 0
    # end_idx = 10
    end_idx = len(imu_data.timestamps) - 1
    imu_buf = imu_data.form_imu_buffer(start_idx, end_idx)

    # Pose i
    ts_i = imu_buf.ts[start_idx]
    T_WS_i = imu_data.poses[ts_i]

    # Speed and bias i
    ts_i = imu_buf.ts[start_idx]
    vel_i = imu_data.vel[ts_i]
    ba_i = np.array([0.0, 0.0, 0.0])
    bg_i = np.array([0.0, 0.0, 0.0])
    sb_i = speed_biases_setup(ts_i, vel_i, bg_i, ba_i)

    # Propagate imu measurements
    data = imu_factor_propagate(imu_buf, imu_params, sb_i)

    dT = tf(data.dC, data.dr)
    T_WS_j_est = T_WS_i @ dT
    ts_j = imu_buf.ts[-1]
    T_WS_j_gnd = imu_data.poses[ts_j]
    print(np.round(T_WS_j_est, 4))
    print(np.round(T_WS_j_gnd, 4))

    print(f"dr: {data.dr}")
    print(f"dv: {data.dv}")
    print(f"dC: {data.dC}")
    print(f"ba: {data.ba}")
    print(f"bg: {data.bg}")
    print(f"Dt: {data.Dt}")

  def test_imu_factor(self):
    """ Test IMU factor """
    # Simulate imu data
    circle_r = 1.0
    circle_v = 1.0
    sim_data = SimData(circle_r, circle_v, sim_cams=False)
    imu_data = sim_data.imu0_data

    # Setup imu parameters
    noise_acc = 0.08  # accelerometer measurement noise stddev.
    noise_gyr = 0.004  # gyroscope measurement noise stddev.
    noise_ba = 0.00004  # accelerometer bias random work noise stddev.
    noise_bg = 2.0e-6  # gyroscope bias random work noise stddev.
    imu_params = ImuParams(noise_acc, noise_gyr, noise_ba, noise_bg)

    # Setup imu buffer
    start_idx = 0
    end_idx = 10
    imu_buf = imu_data.form_imu_buffer(start_idx, end_idx)

    # Pose i
    ts_i = imu_buf.ts[start_idx]
    T_WS_i = imu_data.poses[ts_i]
    pose_i = pose_setup(ts_i, T_WS_i)

    # Pose j
    ts_j = imu_buf.ts[end_idx - 1]
    T_WS_j = imu_data.poses[ts_j]
    pose_j = pose_setup(ts_j, T_WS_j)

    # Speed and bias i
    vel_i = imu_data.vel[ts_i]
    ba_i = np.array([0.0, 0.0, 0.0])
    bg_i = np.array([0.0, 0.0, 0.0])
    sb_i = speed_biases_setup(ts_i, vel_i, bg_i, ba_i)

    # Speed and bias j
    vel_j = imu_data.vel[ts_j]
    ba_j = np.array([0.0, 0.0, 0.0])
    bg_j = np.array([0.0, 0.0, 0.0])
    sb_j = speed_biases_setup(ts_j, vel_j, bg_j, ba_j)

    # Setup IMU factor
    param_ids = [0, 1, 2, 3]
    factor = imu_factor_setup(param_ids, imu_buf, imu_params, sb_i)

    # Evaluate factor
    params = [pose_i, sb_i, pose_j, sb_j]
    # r, jacs = factor.eval(params)
    factor.eval(params)

    # Test jacobians
    self.assertTrue(check_factor_jacobian(factor, params, 0, "J_pose_i"))
    self.assertTrue(check_factor_jacobian(factor, params, 1, "J_sb_i"))
    self.assertTrue(check_factor_jacobian(factor, params, 2, "J_pose_j"))
    self.assertTrue(check_factor_jacobian(factor, params, 3, "J_sb_j"))


class TestFactorGraph(unittest.TestCase):
  """ Test Factor Graph """

  def test_factor_graph_add_param(self):
    """ Test FactorGrpah.add_param() """
    # Setup camera pose T_WC
    rot = euler2quat(-pi / 2.0, 0.0, -pi / 2.0)
    trans = np.array([0.1, 0.2, 0.3])
    T_WC = tf(rot, trans)
    pose0 = pose_setup(0, T_WC)
    pose1 = pose_setup(1, T_WC)

    # Add params
    graph = FactorGraph()
    pose0_id = graph.add_param(pose0)
    pose1_id = graph.add_param(pose1)

    # Assert
    self.assertEqual(pose0_id, 0)
    self.assertEqual(pose1_id, 1)
    self.assertNotEqual(pose0, pose1)
    self.assertEqual(graph.params[pose0_id], pose0)
    self.assertEqual(graph.params[pose1_id], pose1)

  def test_factor_graph_add_factor(self):
    """ Test FactorGrpah.add_factor() """
    # Setup factor graph
    graph = FactorGraph()

    # Setup camera pose T_WC
    rot = euler2quat(-pi / 2.0, 0.0, -pi / 2.0)
    trans = np.array([0.1, 0.2, 0.3])
    T_WC = tf(rot, trans)
    pose = pose_setup(0, T_WC)
    pose_id = graph.add_param(pose)

    # Create factor
    param_ids = [pose_id]
    pose_factor = pose_factor_setup(param_ids, T_WC)
    pose_factor_id = graph.add_factor(pose_factor)

    # Assert
    self.assertEqual(len(graph.params), 1)
    self.assertEqual(len(graph.factors), 1)
    self.assertEqual(graph.factors[pose_factor_id], pose_factor)

  def test_factor_graph_solve_vo(self):
    """ Test FactorGraph.solve() """
    # Sim data
    circle_r = 5.0
    circle_v = 1.0
    pickle_path = '/tmp/sim_data.pickle'
    sim_data = SimData.create_or_load(circle_r, circle_v, pickle_path)

    # Setup factor graph
    poses_init = []
    poses_est = []
    graph = FactorGraph()

    # -- Add features
    features = sim_data.features
    feature_ids = []
    for i in range(features.shape[0]):
      p_W = features[i, :]
      p_W += np.random.rand(3) * 0.1  # perturb feature
      feature = feature_setup(p_W)
      feature_ids.append(graph.add_param(feature))

    # -- Add cam0
    cam0_data = sim_data.mcam_data[0]
    cam0_params = cam0_data.camera
    cam0_geom = cam0_params.data
    cam0_id = graph.add_param(cam0_params)

    # -- Build bundle adjustment problem
    nb_poses = 0
    for ts in cam0_data.timestamps:
      # Camera frame at ts
      cam_frame = cam0_data.frames[ts]

      # Add camera pose T_WC0
      T_WC0 = cam0_data.poses[ts]
      # -- Perturb camera pose
      trans_rand = np.random.rand(3)
      rvec_rand = np.random.rand(3) * 0.1
      T_WC0 = tf_update(T_WC0, np.block([*trans_rand, *rvec_rand]))
      # -- Add to graph
      pose = pose_setup(ts, T_WC0)
      pose_id = graph.add_param(pose)
      poses_init.append(T_WC0)
      poses_est.append(pose)
      nb_poses += 1

      # Add ba factors
      for i, idx in enumerate(cam_frame.feature_ids):
        z = cam_frame.measurements[i]
        param_ids = [pose_id, feature_ids[idx], cam0_id]
        graph.add_factor(ba_factor_setup(param_ids, z, cam0_geom))

    # Solve
    # prof = profile_start()
    graph.solve()
    # profile_stop(prof)

    pos_init = np.array([tf_trans(T) for T in poses_init])
    pos_est = np.array([tf_trans(pose2tf(pose.param)) for pose in poses_est])

    plt.figure()
    plt.plot(pos_init[:, 0], pos_init[:, 1], 'r-')
    plt.plot(pos_est[:, 0], pos_est[:, 1], 'b-')
    plt.xlabel("Displacement [m]")
    plt.ylabel("Displacement [m]")
    plt.show()

    # Asserts
    errors = graph._get_reproj_errors()
    self.assertTrue(rmse(errors) < 0.1)

  def test_factor_graph_solve_vio(self):
    """ Test FactorGraph.solve() """
    # Sim data
    circle_r = 5.0
    circle_v = 1.0
    pickle_path = '/tmp/sim_data.pickle'
    sim_data = SimData.create_or_load(circle_r, circle_v, pickle_path)

    # Imu params
    noise_acc = 0.08  # accelerometer measurement noise stddev.
    noise_gyr = 0.004  # gyroscope measurement noise stddev.
    noise_ba = 0.00004  # accelerometer bias random work noise stddev.
    noise_bg = 2.0e-6  # gyroscope bias random work noise stddev.
    imu_params = ImuParams(noise_acc, noise_gyr, noise_ba, noise_bg)

    # Setup factor graph
    imu0_data = sim_data.imu0_data
    start_idx = 0
    end_idx = 1000
    # end_idx = len(imu0_data.timestamps) - 2

    poses_init = []
    poses_est = []
    graph = FactorGraph()

    # -- Pose i
    ts_i = imu0_data.timestamps[start_idx]
    T_WS_i = imu0_data.poses[ts_i]
    # trans_rand = np.random.rand(3)
    # rvec_rand = np.random.rand(3) * 0.1
    # T_WS_i = tf_update(T_WS_i, np.block([*trans_rand, *rvec_rand]))
    pose_i = pose_setup(ts_i, T_WS_i)
    pose_i_id = graph.add_param(pose_i)
    poses_init.append(T_WS_i)
    poses_est.append(pose_i)

    # -- Speed and biases i
    vel_i = imu0_data.vel[ts_i]
    ba_i = np.array([0.0, 0.0, 0.0])
    bg_i = np.array([0.0, 0.0, 0.0])
    sb_i = speed_biases_setup(ts_i, vel_i, ba_i, bg_i)
    sb_i_id = graph.add_param(sb_i)

    # print(pose2tf(pose_i.param))
    # print(sb_i.param)

    window_size = 10
    for ts_idx in range(start_idx + window_size, end_idx, window_size):
      # -- Pose j
      ts_j = imu0_data.timestamps[ts_idx]
      T_WS_j = imu0_data.poses[ts_j]
      trans_rand = np.random.rand(3)
      rvec_rand = np.random.rand(3) * 0.1
      T_WS_j = tf_update(T_WS_j, np.block([*trans_rand, *rvec_rand]))
      pose_j = pose_setup(ts_j, T_WS_j)
      pose_j_id = graph.add_param(pose_j)
      poses_init.append(T_WS_j)
      poses_est.append(pose_j)

      # -- Speed and biases j
      vel_j = imu0_data.vel[ts_j]
      ba_j = np.array([0.0, 0.0, 0.0])
      bg_j = np.array([0.0, 0.0, 0.0])
      sb_j = speed_biases_setup(ts_j, vel_j, ba_j, bg_j)
      sb_j_id = graph.add_param(sb_j)

      # -- Imu Factor
      param_ids = [pose_i_id, sb_i_id, pose_j_id, sb_j_id]
      imu_buf = imu0_data.form_imu_buffer(ts_idx - window_size, ts_idx)
      factor = imu_factor_setup(param_ids, imu_buf, imu_params, sb_i)
      graph.add_factor(factor)

      # -- Update
      pose_i = pose_j
      sb_i = sb_j

    # # Solve
    # # prof = profile_start()
    graph.solver_max_iter = 100
    graph.solve()
    # # profile_stop(prof)

    pos_init = np.array([tf_trans(T) for T in poses_init])
    pos_est = np.array([tf_trans(pose2tf(pose.param)) for pose in poses_est])

    plt.figure()
    plt.plot(pos_init[:, 0], pos_init[:, 1], 'r-')
    plt.plot(pos_est[:, 0], pos_est[:, 1], 'b-')
    plt.xlabel("Displacement [m]")
    plt.ylabel("Displacement [m]")
    plt.show()


class TestFeatureTracking(unittest.TestCase):
  """ Test feature tracking functions """

  def setUp(self):
    # Load test data
    self.dataset = EurocDataset(euroc_data_path)

    # Setup test images
    ts = self.dataset.timestamps[800]
    self.img0 = cv2.imread(self.dataset.cam0_images[ts], cv2.IMREAD_GRAYSCALE)
    self.img1 = cv2.imread(self.dataset.cam1_images[ts], cv2.IMREAD_GRAYSCALE)

  def test_spread_keypoints(self):
    """ Test spread_keypoints() """
    # img = np.zeros((140, 160))
    # kps = []
    # kps.append(cv2.KeyPoint(10, 10, 0, 0.0, 0.0, 0))
    # kps.append(cv2.KeyPoint(150, 130, 0, 0.0, 0.0, 1))
    # kps = spread_keypoints(img, kps, 5, debug=True)

    detector = cv2.FastFeatureDetector_create(threshold=50)
    kwargs = {'optflow_mode': True, 'debug': False}
    kps = grid_detect(detector, self.img0, **kwargs)
    kps = spread_keypoints(self.img0, kps, 20, debug=False)

    self.assertTrue(len(kps))

  def test_feature_grid_cell_index(self):
    """ Test FeatureGrid.grid_cell_index() """
    grid_rows = 4
    grid_cols = 4
    image_shape = (320, 280)
    keypoints = [[0, 0], [320, 0], [0, 280], [320, 280]]
    grid = FeatureGrid(grid_rows, grid_cols, image_shape, keypoints)

    self.assertEqual(grid.cell[0], 1)
    self.assertEqual(grid.cell[3], 1)
    self.assertEqual(grid.cell[12], 1)
    self.assertEqual(grid.cell[15], 1)

  def test_feature_grid_count(self):
    """ Test FeatureGrid.count() """
    grid_rows = 4
    grid_cols = 4
    image_shape = (320, 280)
    pts = [[0, 0], [320, 0], [0, 280], [320, 280]]
    grid = FeatureGrid(grid_rows, grid_cols, image_shape, pts)

    self.assertEqual(grid.count(0), 1)
    self.assertEqual(grid.count(3), 1)
    self.assertEqual(grid.count(12), 1)
    self.assertEqual(grid.count(15), 1)

  def test_grid_detect(self):
    """ Test grid_detect() """
    debug = False

    # detector = cv2.ORB_create(nfeatures=500)
    # kps, des = grid_detect(detector, self.img0, **kwargs)
    # self.assertTrue(len(kps) > 0)
    # self.assertEqual(des.shape[0], len(kps))

    detector = cv2.FastFeatureDetector_create(threshold=50)
    kwargs = {'optflow_mode': True, 'debug': debug}
    kps = grid_detect(detector, self.img0, **kwargs)
    self.assertTrue(len(kps) > 0)

  def test_optflow_track(self):
    """ Test optflow_track() """
    debug = True

    # Detect
    feature = cv2.ORB_create(nfeatures=100)
    kps, des = grid_detect(feature, self.img0)
    self.assertTrue(len(kps) == len(des))

    # Track
    pts_i = np.array([kp.pt for kp in kps], dtype=np.float32)
    track_results = optflow_track(self.img0, self.img1, pts_i, debug=debug)
    (pts_i, pts_j, inliers) = track_results

    self.assertTrue(len(pts_i) == len(pts_j))
    self.assertTrue(len(pts_i) == len(inliers))


class TestFeatureTracker(unittest.TestCase):
  """ Test FeatureTracker """

  def setUp(self):
    # Load test data
    self.dataset = EurocDataset(euroc_data_path)

    # Setup test images
    ts = self.dataset.timestamps[0]
    img0_path = self.dataset.cam0_images[ts]
    img1_path = self.dataset.cam1_images[ts]
    self.img0 = cv2.imread(img0_path, cv2.IMREAD_GRAYSCALE)
    self.img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)

    # Setup cameras
    # -- cam0
    res = self.dataset.cam0_config.resolution
    proj_params = self.dataset.cam0_config.intrinsics
    dist_params = self.dataset.cam0_config.distortion_coefficients
    proj_model = "pinhole"
    dist_model = "radtan4"
    params = np.block([*proj_params, *dist_params])
    cam0 = camera_params_setup(0, res, proj_model, dist_model, params)
    # -- cam1
    res = self.dataset.cam1_config.resolution
    proj_params = self.dataset.cam1_config.intrinsics
    dist_params = self.dataset.cam1_config.distortion_coefficients
    proj_model = "pinhole"
    dist_model = "radtan4"
    params = np.block([*proj_params, *dist_params])
    cam1 = camera_params_setup(1, res, proj_model, dist_model, params)

    # Setup camera extrinsics
    # -- cam0
    T_BC0 = self.dataset.cam0_config.T_BS
    cam0_exts = extrinsics_setup(T_BC0)
    # -- cam1
    T_BC1 = self.dataset.cam1_config.T_BS
    cam1_exts = extrinsics_setup(T_BC1)

    # Setup feature tracker
    self.feature_tracker = FeatureTracker()
    self.feature_tracker.add_camera(0, cam0, cam0_exts)
    self.feature_tracker.add_camera(1, cam1, cam1_exts)
    self.feature_tracker.add_overlap(0, 1)

  def test_detect(self):
    """ Test FeatureTracker._detect() """
    # Load and detect features from single image
    kps = self.feature_tracker._detect(self.img0)
    self.assertTrue(len(kps) > 0)

  def test_detect_overlaps(self):
    """ Test FeatureTracker._detect_overlaps() """
    debug = False

    # Feed camera images to feature tracker
    camera_images = {}
    camera_images[0] = self.img0
    camera_images[1] = self.img1
    self.feature_tracker._detect_overlaps(camera_images)

    # Assert
    data_i = self.feature_tracker.cam_data[0]
    data_j = self.feature_tracker.cam_data[1]
    kps_i = data_i.keypoints
    kps_j = data_j.keypoints
    overlapping_ids = self.feature_tracker.feature_overlaps

    self.assertTrue(len(kps_i) == len(kps_j))
    self.assertTrue(len(kps_i) == len(overlapping_ids))

    # Visualize
    for cam_i, cam_j in self.feature_tracker.cam_overlaps:
      img_i = camera_images[cam_i]
      img_j = camera_images[cam_j]
      data_i = self.feature_tracker.cam_data[cam_i]
      data_j = self.feature_tracker.cam_data[cam_j]
      kps_i = data_i.keypoints
      kps_j = data_j.keypoints
      # viz = draw_matches(img_i, img_j, kps_i, kps_j)

      matches = []
      for i in range(len(kps_i)):
        matches.append(cv2.DMatch(i, i, 0))
      viz = cv2.drawMatches(img_i, kps_i, img_j, kps_j, matches, None)

      if debug:
        cv2.imshow('viz', viz)
        cv2.waitKey(0)

  def test_detect_nonoverlaps(self):
    """ Test FeatureTracker._detect_nonoverlaps() """
    debug = True

    # Feed camera images to feature tracker
    camera_images = {}
    camera_images[0] = self.img0
    camera_images[1] = self.img1
    self.feature_tracker._detect_nonoverlaps(camera_images)

    # Visualize
    for cam_i, cam_j in self.feature_tracker.cam_overlaps:
      img_i = camera_images[cam_i]
      img_j = camera_images[cam_j]
      data_i = self.feature_tracker.cam_data[cam_i]
      data_j = self.feature_tracker.cam_data[cam_j]
      kps_i = data_i.keypoints
      kps_j = data_j.keypoints

      viz_i = cv2.drawKeypoints(img_i, kps_i, None)
      viz_j = cv2.drawKeypoints(img_j, kps_j, None)
      viz = cv2.hconcat([viz_i, viz_j])

      if debug:
        cv2.imshow('viz', viz)
        cv2.waitKey(0)

  def test_detect_new(self):
    """ Test FeatureTracker.detect_new() """
    debug = True

    camera_images = {}
    camera_images[0] = self.img0
    camera_images[1] = self.img1
    viz = self.feature_tracker._detect_new(camera_images)

    if debug:
      cv2.imshow('viz', viz)
      cv2.waitKey(0)

  def test_update(self):
    """ Test FeatureTracker.update() """
    for ts in self.dataset.timestamps[1000:]:
      # Load images
      img0_path = self.dataset.cam0_images[ts]
      img1_path = self.dataset.cam1_images[ts]
      img0 = cv2.imread(img0_path, cv2.IMREAD_GRAYSCALE)
      img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)

      # Feed camera images to feature tracker
      camera_images = {}
      camera_images[0] = img0
      camera_images[1] = img1
      ft_data = self.feature_tracker.update(ts, camera_images)
      viz = visualize_tracking(ft_data)

      # Visualize
      sys.stdout.flush()
      cv2.imshow('viz', viz)
      # if cv2.waitKey(0) == ord('q'):
      #   break
      if cv2.waitKey(1) == ord('q'):
        break


class TestStateEstimator(unittest.TestCase):
  """ Test State Estimator """

  def setUp(self):
    # Load test data
    self.dataset = EurocDataset(euroc_data_path)

    # Setup test images
    ts = self.dataset.timestamps[0]
    img0_path = self.dataset.cam0_images[ts]
    img1_path = self.dataset.cam1_images[ts]
    self.img0 = cv2.imread(img0_path, cv2.IMREAD_GRAYSCALE)
    self.img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)

    # Setup cameras
    # -- cam0
    res = self.dataset.cam0_config.resolution
    proj_params = self.dataset.cam0_config.intrinsics
    dist_params = self.dataset.cam0_config.distortion_coefficients
    proj_model = "pinhole"
    dist_model = "radtan4"
    params = np.block([*proj_params, *dist_params])
    self.cam0 = camera_params_setup(0, res, proj_model, dist_model, params)
    # -- cam1
    res = self.dataset.cam1_config.resolution
    proj_params = self.dataset.cam1_config.intrinsics
    dist_params = self.dataset.cam1_config.distortion_coefficients
    proj_model = "pinhole"
    dist_model = "radtan4"
    params = np.block([*proj_params, *dist_params])
    self.cam1 = camera_params_setup(1, res, proj_model, dist_model, params)

    # Setup camera extrinsics
    # -- cam0
    T_BC0 = self.dataset.cam0_config.T_BS
    self.cam0_exts = extrinsics_setup(T_BC0)
    # -- cam1
    T_BC1 = self.dataset.cam1_config.T_BS
    self.cam1_exts = extrinsics_setup(T_BC1)

    # Setup tracker
    self.tracker = Tracker()
    self.tracker.add_camera(0, self.cam0, self.cam0_exts)
    self.tracker.add_camera(1, self.cam1, self.cam1_exts)
    self.tracker.add_overlap(0, 1)

  def test_tracker_vision_callback(self):
    """ Test Tracker.vision_callback() """
    for ts in self.dataset.timestamps[100:]:
      # Load images
      img0_path = self.dataset.cam0_images[ts]
      img1_path = self.dataset.cam1_images[ts]
      img0 = cv2.imread(img0_path, cv2.IMREAD_GRAYSCALE)
      img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)

      # Feed camera images to feature tracker
      ft_data = self.tracker.vision_callback(ts, {0: img0, 1: img1})
      viz = visualize_tracking(ft_data)

      # Visualize
      sys.stdout.flush()
      cv2.imshow('viz', viz)
      # if cv2.waitKey(0) == ord('q'):
      #   break
      if cv2.waitKey(1) == ord('q'):
        break


# CALIBRATION #################################################################


class TestCalibration(unittest.TestCase):
  """ Test calibration functions """

  def test_dummy(self):
    """ Test dummy """
    pass


# DATASET  ####################################################################


class TestEuoc(unittest.TestCase):
  """ Test Euroc dataset loader """

  def test_load(self):
    """ Test load """
    data_path = '/data/euroc/raw/V1_01'
    dataset = EurocDataset(data_path)
    self.assertTrue(dataset is not None)


# SIMULATION  #################################################################


class TestSimulation(unittest.TestCase):
  """ Test simulation functions """

  def test_create_3d_features(self):
    """ Test create 3D features """
    debug = False
    x_bounds = np.array([-10.0, 10.0])
    y_bounds = np.array([-10.0, 10.0])
    z_bounds = np.array([-10.0, 10.0])
    nb_features = 1000
    features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_features)
    self.assertTrue(features.shape == (nb_features, 3))

    if debug:
      fig = plt.figure()
      ax = fig.gca(projection='3d')
      ax.scatter(features[:, 0], features[:, 1], features[:, 2])
      ax.set_xlabel("x [m]")
      ax.set_ylabel("y [m]")
      ax.set_zlabel("z [m]")
      plt.show()

  def test_create_3d_features_perimeter(self):
    """ Test create_3d_features_perimeter() """
    debug = False
    origin = np.array([0.0, 0.0, 0.0])
    dim = np.array([10.0, 10.0, 5.0])
    nb_features = 1000
    features = create_3d_features_perimeter(origin, dim, nb_features)
    self.assertTrue(features.shape == (nb_features, 3))

    if debug:
      fig = plt.figure()
      ax = fig.gca(projection='3d')
      ax.scatter(features[:, 0], features[:, 1], features[:, 2])
      ax.set_xlabel("x [m]")
      ax.set_ylabel("y [m]")
      ax.set_zlabel("z [m]")
      plt.show()

  def test_sim_data(self):
    """ Test SimData() """
    debug_cam = False
    debug_imu = False

    # Sim data
    circle_r = 5.0
    circle_v = 1.0
    pickle_path = '/tmp/sim_data.pickle'
    sim_data = SimData.create_or_load(circle_r, circle_v, pickle_path)
    cam0_data = sim_data.mcam_data[0]
    cam1_data = sim_data.mcam_data[1]

    self.assertTrue(sim_data is not None)
    self.assertTrue(sim_data.features.shape[0] > 0)
    self.assertTrue(sim_data.features.shape[1] == 3)
    self.assertTrue(cam0_data.cam_idx == 0)
    self.assertTrue(len(cam0_data.poses) == len(cam0_data.frames))
    self.assertTrue(cam1_data.cam_idx == 1)
    self.assertTrue(len(cam1_data.poses) == len(cam1_data.frames))

    if debug_cam:
      cam0_data = sim_data.mcam_data[0]
      pos = np.array([tf_trans(v) for k, v in cam0_data.poses.items()])

      plt.figure()
      plt.plot(pos[:, 0], pos[:, 1], 'r-')
      plt.xlabel("Displacement [m]")
      plt.ylabel("Displacement [m]")
      plt.title("Camera Position")
      plt.subplots_adjust(hspace=0.9)
      plt.show()

    if debug_imu:
      imu0_data = sim_data.imu0_data

      pos = np.array([tf_trans(v) for k, v in imu0_data.poses.items()])
      vel = np.array([v for k, v in imu0_data.vel.items()])
      acc = np.array([v for k, v in imu0_data.acc.items()])
      gyr = np.array([v for k, v in imu0_data.gyr.items()])

      plt.figure()
      plt.subplot(411)
      plt.plot(pos[:, 0], pos[:, 1], 'r-')
      plt.xlabel("Time [s]")
      plt.ylabel("Displacement [m]")
      plt.title("IMU Position")

      plt.subplot(412)
      plt.plot(imu0_data.timestamps, vel[:, 0], 'r-')
      plt.plot(imu0_data.timestamps, vel[:, 1], 'g-')
      plt.plot(imu0_data.timestamps, vel[:, 2], 'b-')
      plt.xlabel("Time [s]")
      plt.ylabel("Velocity [ms^-1]")
      plt.title("IMU Velocity")

      plt.subplot(413)
      plt.plot(imu0_data.timestamps, acc[:, 0], 'r-')
      plt.plot(imu0_data.timestamps, acc[:, 1], 'g-')
      plt.plot(imu0_data.timestamps, acc[:, 2], 'b-')
      plt.xlabel("Time [s]")
      plt.ylabel("Acceleration [ms^-2]")
      plt.title("Accelerometer Measurements")

      plt.subplot(414)
      plt.plot(imu0_data.timestamps, gyr[:, 0], 'r-')
      plt.plot(imu0_data.timestamps, gyr[:, 1], 'g-')
      plt.plot(imu0_data.timestamps, gyr[:, 2], 'b-')
      plt.xlabel("Time [s]")
      plt.ylabel("Angular Velocity [rad s^-1]")
      plt.title("Gyroscope Measurements")

      plt.subplots_adjust(hspace=0.9)
      plt.show()


if __name__ == '__main__':
  unittest.main()
