import os
import sys
import glob
import math
from dataclasses import dataclass
from typing import Optional


import cv2
import yaml
import numpy as np

###############################################################################
# MATHS
###############################################################################

from math import isclose
from math import cos
from math import sin
from math import sqrt
from math import pi

###############################################################################
# LINEAR ALGEBRA
###############################################################################

from numpy import rad2deg
from numpy import deg2rad
from numpy import zeros
from numpy import ones
from numpy import eye
from numpy import diagonal as diag
from numpy.linalg import norm
from numpy.linalg import inv
from numpy.linalg import pinv
from numpy.linalg import matrix_rank as rank
from numpy.linalg import eig


def normalize(v):
  norm = np.linalg.norm(v)
  if norm == 0:
    return v
  return v / norm


def fullrank(A):
  return rank(A) == A.shape[0]


def skew(vec):
  assert vec.shape == (3,) or vec.shape == (3, 1)
  x, y, z = vec
  return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def skew_inv(A):
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
  if (precond):
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
  diff = A - B

  if len(diff.shape) == 1:
    for i in range(len(diff)):
      if (abs(diff[i]) > tol):
        if verbose:
          print("A - B:")
          print(diff)

  elif len(diff.shape) == 2:
    for i in range(diff.shape[0]):
      for j in range(diff.shape[1]):
        if (abs(diff[i, j]) > tol):
          if verbose:
            print("A - B:")
            print(diff)
          return False

  return True


def check_jacobian(jac_name, fdiff, jac, threshold, verbose=False):
  failed = False
  d = (fdiff - jac)

  # Compare matrices
  for i in range(d.shape[0]):
    for j in range(d.shape[1]):
      delta = d[i, j]
      if (abs(delta) > threshold):
        failed = True

  if failed:
    retval = -1
    if verbose:
      print("Check [%s] failed!" % jac_name)
    fdiff_minus_jac = fdiff - jac
    num_diff = fdiff

    # delta
    if verbose:
      print("----------------------------------------")
  else:
    if verbose:
      print("Check [%s] passed!" % jac_name)
    retval = 0

  return retval


###############################################################################
# LIE
###############################################################################


def Exp(phi):
  assert phi.shape == (3,) or phi.shape == (3, 1)
  if (phi < 1e-3):
    C = eye(3) + skew(phi)
    return C
  else:
    phi_norm = norm(phi)
    phi_skew = skew(phi)
    phi_skew_sq = phi_skew * phi_skew

    C = eye(3)
    C += (sin(phi_norm) / phi_norm) * phi_skew
    C += ((1 - cos(phi_norm)) / phi_norm ^ 2) * phi_skew_sq
    return C


def Log(C):
  assert C.shape == (3, 3)
  # phi = acos((trace(C) - 1) / 2);
  # u = skew_inv(C - C') / (2 * sin(phi));
  # rvec = phi * u;

  C00, C01, C02 = C[0, :]
  C10, C11, C12 = C[1, :]
  C20, C21, C22 = C[2, :]

  tr = np.trace(C)
  if (tr + 1.0 < 1e-10):
    if (abs(C22 + 1.0) > 1e-5):
      x = np.array([C02, C12, 1.0 + C22])
      rvec = (math.pi / np.sqrt(2.0 + 2.0 * C22)) * x
    elif (abs(C11 + 1.0) > 1e-5):
      x = np.array([C01, 1.0 + C11, C21])
      rvec = (math.pi / np.sqrt(2.0 + 2.0 * C11)) * x
    else:
      x = np.array([1.0 + C00, C10, C20])
      rvec = (math.pi / np.sqrt(2.0 + 2.0 * C00)) * x
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


def Jr(theta):
  """
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


def boxminus(C_a, C_b):
  # alpha = C_a [-] C_b
  alpha = Log(inv(C_b) * C_a)
  return alpha


def boxplus(C, alpha):
  # C_updated = C [+] alpha
  C_updated = C * Exp(alpha)
  return C_updated


###############################################################################
# TRANSFORM
###############################################################################


def homogeneous(p):
  return np.array([*p, 1.0])


def dehomogeneous(hp):
  return hp[0:3]


def rotx(theta):
  row0 = [1.0, 0.0, 0.0]
  row1 = [0.0, cos(theta), -sin(theta)]
  row2 = [0.0, sin(theta), cos(theta)]
  return np.array([row0, row1, row2])


def roty(theta):
  row0 = [cos(theta), 0.0, sin(theta)]
  row1 = [0.0, 1.0, 0.0]
  row2 = [-sin(theta), 0.0, cos(theta)]
  return np.array([row0, row1, row2])


def rotz(theta):
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
  # If small rotation
  theta = sqrt(rvec @ rvec)  # = norm(rvec), but faster
  if theta < eps:
    # yapf: disable
    R = np.array([[1.0, -rvec[2], rvec[1]],
                  [rvec[2], 1.0, -rvec[0]],
                  [-rvec[1], rvec[0], 1.0]])
    # yapf: enable
    return R

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

  # yapf: disable
  R = np.array([[x * xC + c, xyC - zs, zxC + ys],
                [xyC + zs, y * yC + c, yzC - xs],
                [zxC - ys, yzC + xs, z * zC + c]])
  # yapf: enable
  return R


def vecs2axisangle(u, v):
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

  qw2 = qw**2
  qx2 = qx**2
  qy2 = qy**2
  qz2 = qz**2

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
  C12 = 2 * (qx * qy - qw * qz)
  C13 = 2 * (qx * qz + qw * qy)

  C21 = 2 * (qx * qy + qw * qz)
  C22 = qw2 - qx2 + qy2 - qz2
  C23 = 2 * (qy * qz - qw * qx)

  C31 = 2 * (qx * qz - qw * qy)
  C32 = 2 * (qy * qz + qw * qx)
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

  m00 = R[0, 0]
  m01 = R[0, 1]
  m02 = R[0, 2]

  m10 = R[1, 0]
  m11 = R[1, 1]
  m12 = R[1, 2]

  m20 = R[2, 0]
  m21 = R[2, 1]
  m22 = R[2, 2]

  tr = m00 + m11 + m22

  if (tr > 0):
    S = sqrt(tr + 1.0) * 2
    # S=4*qw
    qw = 0.25 * S
    qx = (m21 - m12) / S
    qy = (m02 - m20) / S
    qz = (m10 - m01) / S
  elif ((m00 > m11) and (m00 > m22)):
    S = sqrt(1.0 + m00 - m11 - m22) * 2
    # S=4*qx
    qw = (m21 - m12) / S
    qx = 0.25 * S
    qy = (m01 + m10) / S
    qz = (m02 + m20) / S
  elif (m11 > m22):
    S = sqrt(1.0 + m11 - m00 - m22) * 2
    # S=4*qy
    qw = (m02 - m20) / S
    qx = (m01 + m10) / S
    qy = 0.25 * S
    qz = (m12 + m21) / S
  else:
    S = sqrt(1.0 + m22 - m00 - m11) * 2
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
  # yapf: disable
  L = np.array([qw, -qx, -qy, -qz,
                qx, qw, -qz, qy,
                qy, qz, qw, -qx,
                qz, -qy, qx, qw]).reshape((4, 4))
  # yapf: enable
  return L


def quat_right(q):
  """ Quaternion right product matrix """
  qw, qx, qy, qz = q
  # yapf: disable
  R = np.array([qw, -qx, -qy, -qz,
                qx, qw, qz, -qy,
                qy, -qz, qw, qx,
                qz, qy, -qx, qw]).reshape((4, 4))
  # yapf: enable
  return R


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
  # Omega = [-skew(w), w;
  #          -transpose(w), 0.0];
  pass


def quat_delta(dalpha):
  """ Form quaternion from small angle rotation vector dalpha """
  half_norm = 0.5 * norm(dalpha)
  scalar = cos(half_norm)
  vector = sinc(half_norm) * 0.5 * dalpha
  dq = np.array([scalar, vector])  # (qw, qx, qy, qz)
  return dq


def quat_integrate(q_k, w, dt):
  """
  "Quaternion kinematics for the error-state Kalman filter" (2017)
  By Joan Sola
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
  return T[0:3, 0:3]


def tf_quat(T):
  """ Return quaternion from 4x4 homogeneous transform """
  return rot2quat(tf_rot(T))


def tf_inv(T):
  """ Invert 4x4 homogeneous transform """
  return np.linalg.inv(T)


def tf_point(T, p):
  """ Transform 3d point """
  assert p.shape == (3,) or p.shape == (3, 1)
  hpoint = np.array([p[0], p[1], p[2], 1.0])
  return (T @ hpoint)[0:3]


def tf_hpoint(T, hp):
  """ Transform 3d point """
  assert hp.shape == (4,) or hp.shape == (4, 1)
  return (T @ hpoint)[0:3]


def tf_decompose(tf):
  """ Decompose into rotation matrix and translation vector"""
  C = tf_rot(tf)
  r = tf_trans(tf)
  return (C, r)


def tf_vector(T):
  """ Return pose vector """
  rx, ry, rz = tf_trans(T)
  qw, qx, qy, qz = tf_quat(T)
  return np.array([rx, ry, rz, qw, qx, qy, qz])

###############################################################################
# MATPLOTLIB
###############################################################################

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
  size = kwargs.get('size', 1)
  linewidth = kwargs.get('linewidth', 3)
  name = kwargs.get('name', None)
  name_offset = kwargs.get('name_offset', [0, 0, -0.01])
  fontsize = kwargs.get('fontsize', 10)
  fontweight = kwargs.get('fontweight', 'bold')
  colors = kwargs.get('colors', ['r-', 'g-', 'b-'])

  p_f = np.array([[0, 0, 0, 1], [size, 0, 0, 1], [0, size, 0, 1], [0, 0, size, 1]]).T
  p_0 = T @ p_f
  X = np.append([p_0[:, 0].T], [p_0[:, 1].T], axis=0)
  Y = np.append([p_0[:, 0].T], [p_0[:, 2].T], axis=0)
  Z = np.append([p_0[:, 0].T], [p_0[:, 3].T], axis=0)

  ax.plot3D(X[:, 0], X[:, 1], X[:, 2], colors[0], linewidth=linewidth)
  ax.plot3D(Y[:, 0], Y[:, 1], Y[:, 2], colors[1], linewidth=linewidth)
  ax.plot3D(Z[:, 0], Z[:, 1], Z[:, 2], colors[2], linewidth=linewidth)

  if name is not None:
    x = X[0, 0] + name_offset[0]
    y = X[0, 1] + name_offset[1]
    z = X[0, 2] + name_offset[2]
    ax.text(x, y, z, name, fontsize=fontsize, fontweight=fontweight)


###############################################################################
# CV
###############################################################################

# UTILS #######################################################################


def lookat(cam_pos, target_pos, up_axis=[0.0, -1.0, 0.0]):
  """ Form look at matrix """
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
  """

  # First three rows of P_i and P_j
  P1T = P_i[0, :]
  P2T = P_i[1, :]
  P3T = P_i[2, :]
  P1T_dash = P_j[0, :]
  P2T_dash = P_j[1, :]
  P3T_dash = P_j[2, :]

  # Image point from the first and second frame
  x, y = z_i
  x_dash, y_dash = z_j

  # Form the A matrix of AX = 0
  A = zeros((4, 1))
  # A = [y * P3T - P2T;
  #      x * P3T - P1T;
  #      y_dash * P3T_dash - P2T_dash;
  #      x_dash * P3T_dash - P1T_dash];

  # Use SVD to solve AX = 0
  (_, _, V) = svd(A.T * A)
  hp = V[:, 3]  # Get the best result from SVD (last column of V)
  hp = hp / hp[2]  # Normalize the homogeneous 3D point
  p = hp[0:3]  # Return only the first three components (x, y, z)
  return p


# PINHOLE #####################################################################


def pinhole_K(params):
  """ Form camera matrix K """
  fx, fy, cx, cy = params
  return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


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


def pinhole_params_jacobian(proj_params, x):
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


def radtan4_undistort(dist_params, p0):
  """ Un-distort point with Radial-Tangential distortion """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, p1, p2 = dist_params

  # Undistort
  p = p0
  max_iter = 5

  for i in range(max_iter):
    # Error
    p_distorted = radtan4_distort(dist_params, p)
    J = radtan4_point_jacobian(dist_params, p)
    err = (p0 - p_distorted)

    # Update
    # dp = inv(J' * J) * J' * err
    dp = pinv(J) * err
    p = p + dp

    # Check threshold
    if (err.T * err) < 1e-15:
      break

  return p


def radtan4_params_jacobian(dist_params, p):
  """ Radial-Tangential distortion parameter jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, p1, p2 = dist_params

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

  th = thd  # Initial guess
  for i in range(20):
    th2 = th * th
    th4 = th2 * th2
    th6 = th4 * th2
    th8 = th4 * th4
    th = thd / (1.0 + k1() * th2 + k2() * th4 + k3() * th6 + k4() * th8)

  scaling = math.tan(th) / thd
  return np.array([p[0] * scaling, p[1] * scaling])


def equi4_params_jacobian(dist_params, p):
  """ Equi-distant distortion params jacobian """
  assert len(dist_params) == 4
  assert len(p) == 2

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

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
  J[0:2, 0:4] = pinhole_params_jacobian(proj_params, x_dist)
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
  J[0:2, 0:4] = pinhole_params_jacobian(proj_params, x_dist)
  J[0:2, 4:8] = J_proj_point @ J_dist_params
  return J

# CAMERA GEOMETRY #############################################################

class CameraGeometry:
  def __init__(self, cam_idx, resolution, proj_model, dist_model, proj_params,
               dist_params, fix):
    self.fix = fix
    self.type = "camera"
    self.cam_idx = cam_idx
    self.resolution = resolution
    self.proj_model = proj_model
    self.dist_model = dist_model
    self.param = np.array([proj_params, dist_params])
    self.proj_params_size = len(proj_params)
    self.dist_params_size = len(dist_params)
    self.min_dims = self.proj_params + self.dist_params

    if proj_model == "pinhole" and dist_model == "radtan4":
      self.project_fn = pinhole_radtan4_project
      self.backproject_fn = pinhole_radtan4_backproject
      self.J_proj_fn = pinhole_radtan4_project_jacobian
      self.J_param_fn = pinhole_radtan4_params_jacobian
    elif proj_model == "pinhole" and dist_model == "equi4":
      self.project_fn = pinhole_equi4_project
      self.backproject_fn = pinhole_equi4_backproject
      self.J_proj_fn = pinhole_equi4_project_jacobian
      self.J_param_fn = pinhole_equi4_params_jacobian


###############################################################################
# STATE ESTIMATION
###############################################################################

# STATE VARIABLES #############################################################

@dataclass
class StateVariable:
  ts: int
  var_type: str
  param: np.array
  parameterization: str
  min_dims: int
  fix: bool


def pose_setup(ts, param, fix = False):
  param = tf_param(param) if param.shape == (4, 4) else param
  return StateVariable(ts, "pose", param, None, 6, fix)


def extrinsics_setup(param, fix = False):
  param = tf_param(param) if param.shape == (4, 4) else param
  return StateVariable(None, "extrinsics", param, None, 6, fix)


def camera_params_setup(param, fix = False):
  return StateVariable(None, "camera", param, None, len(param), fix)


def feature_setup(param, fix = False):
  return StateVariable(None, "feature", param, None, len(param), fix)



# FEATURE TRACKING #############################################################


def draw_matches(img_i, img_j, kps_i, kps_j):
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
  thickness = 1

  for n in range(nb_kps):
    pt_i = (int(kps_i[n].pt[0]), int(kps_i[n].pt[1]))
    pt_j = (int(kps_j[n].pt[0] + img_i.shape[1]), int(kps_j[n].pt[1]))
    cv2.circle(viz, (pt_i[0], pt_i[1]), radius, color, thickness)
    cv2.circle(viz, (pt_j[0], pt_j[1]), radius, color, thickness)
    cv2.line(viz, pt_i, pt_j, color, 1, 8)

  return viz


def draw_points(img, pts, inliers=None):
  """
  Draw points `pts` on image `img`. The `inliers` boolean list is optional
  and is expected to be the same size as `pts` denoting whether the point
  should be drawn or not.
  """
  if inliers is None:
    inliers = [1 for i in range(len(pts))]

  radius = 3
  color = (0, 255, 0)
  thickness = 1

  viz = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
  for n in range(len(pts)):
    if inliers[n]:
      cv2.circle(viz, (pts[n][0], pts[n][1]), radius, color, thickness)

  return viz


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
      self.cell[self.cell_index(kp)] += 1

  def cell_index(self, kp):
    pixel_x, pixel_y = kp.pt
    img_w, img_h = self.image_shape
    grid_x = math.ceil((max(1, pixel_x) / img_w) * self.grid_cols) - 1.0
    grid_y = math.ceil((max(1, pixel_y) / img_h) * self.grid_rows) - 1.0
    cell_id = int(grid_x + (grid_y * self.grid_cols))
    return cell_id

  def count(self, cell_idx):
    return self.cell[cell_idx]


def grid_detect(detector, image, **kwargs):
  """
  Detect features uniformly using a grid system.
  """
  max_keypoints = kwargs.get('max_keypoints', 1000)
  grid_rows = kwargs.get('grid_rows', 4)
  grid_cols = kwargs.get('grid_cols', 4)
  prev_kps = kwargs.get('prev_kps', [])

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
  for x in range(0, image_width, dx):
    for y in range(0, image_height, dy):
      # Make sure roi width and height are not out of bounds
      w = image_width - x if (x + dx > image_width) else dx
      h = image_height - y if (y + dy > image_height) else dy

      # Detect corners in grid cell
      cs, ce, rs, re = (x, x + w, y, y + h)
      kps = detector.detect(image[rs:re, cs:ce], None)
      kps, des = detector.compute(image, kps)

      # Offset actual keypoint position relative to full image
      cell_vacancy = max_per_cell - feature_grid.count(cell_idx)
      for i in range(min(len(kps), cell_vacancy)):
        kp = kps[i]
        kp.pt = (kp.pt[0] + x, kp.pt[1] + y)
        kps_all.append(kp)
        des_all.append(des[i, :])

      cell_idx += 1

  if kwargs.get('debug', False):
    viz = cv2.drawKeypoints(image, kps_all, None)
    cv2.imshow("viz", viz)
    cv2.waitKey(0)

  return kps_all, np.array(des_all)


def optflow_track(img_i, img_j, pts_i, **kwargs):
  """
  Track keypoints `pts_i` from image `img_i` to image `img_j` using optical
  flow. Returns a tuple of `(pts_i, pts_j, inliers)` points in image i, j and a
  vector of inliers.
  """
  # Track using optical flow
  patch_size = kwargs.get('patch_size', 40)
  max_iter = kwargs.get('max_iter', 100)
  epsilon = kwargs.get('epsilon', 0.01)
  crit = (cv2.TermCriteria_COUNT | cv2.TermCriteria_EPS, max_iter, epsilon)

  config = {}
  config['winSize'] = (patch_size, patch_size)
  config['maxLevel'] = 3
  config['criteria'] = crit
  config['flags'] = cv2.OPTFLOW_USE_INITIAL_FLOW

  pts_j = np.array(pts_i)
  track_results = cv2.calcOpticalFlowPyrLK(img_i, img_j, pts_i, pts_j, **config)
  (pts_j, optflow_mask, errs) = track_results

  # Remove outliers using RANSAC
  F, fundmat_mask = cv2.findFundamentalMat(pts_i, pts_j, cv2.FM_8POINT)

  # Update or mark feature as lost
  inliers = []
  for i in range(len(pts_i)):
    if optflow_mask[i, 0] and fundmat_mask[i, 0]:
      inliers.append(True)
    else:
      inliers.append(False)

  if kwargs.get('debug', False):
    viz_i = draw_points(img_i, pts_i, inliers)
    viz_j = draw_points(img_j, pts_j, inliers)
    viz = cv2.hconcat([viz_i, viz_j])
    cv2.imshow('viz', viz)
    cv2.waitKey(0)

  return (pts_i, pts_j, inliers)


class FeatureTrackingData:
  """ Feature tracking data per camera

  This data structure keeps track of:

  - Image
  - Keypoints
  - Descriptors
  - Feature ids (optional)

  """

  def __init__(self,
               cam_idx,
               image,
               keypoints,
               descriptors=None,
               feature_ids=None):
    self.cam_idx = cam_idx
    self.image = image
    self.keypoints = keypoints
    self.descriptors = descriptors
    self.feature_ids = feature_ids

  def get_all(self):
    return (self.cam_idx, self.image, self.keypoints, self.descriptors,
            self.feature_ids)


class FeatureTracker:
  """
  Feature tracker

  """

  def __init__(self):
    # Settings
    self.mode = "DEFAULT"
    # self.mode = "OVERLAPS_ONLY"
    # self.mode = "NON_OVERLAPS_ONLY"

    # Feature detector, descriptor and matcher
    self.feature = cv2.ORB_create(nfeatures=100)
    self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Data
    self.frame_idx = 0
    self.features_detected = 0
    self.prev_camera_images = None
    self.cam_indices = []
    self.cam_params = {}
    self.cam_exts = {}
    self.cam_overlaps = []
    self.cam_data = {}

  def add_camera(self, cam_idx, cam_params=None, cam_exts=None):
    self.cam_indices.append(cam_idx)
    self.cam_data[cam_idx] = None
    self.cam_params[cam_idx] = cam_params
    self.cam_exts[cam_idx] = cam_exts

  def add_overlap(self, cam_i_idx, cam_j_idx):
    self.cam_overlaps.append((cam_i_idx, cam_j_idx))

  def _get_keypoints(self, cam_idx):
    keypoints = []
    if self.cam_data[cam_idx] is not None:
      keypoints = self.cam_data[cam_idx].keypoints
    return keypoints

  def _form_feature_ids(self, nb_kps):
    self.features_detected += nb_kps
    start_idx = self.features_detected - nb_kps
    end_idx = start_idx + nb_kps
    return range(start_idx, end_idx)

  def _detect(self, cam_idx, image, prev_kps=[]):
    assert image is not None
    # kps, des = grid_detect(self.feature, image, prev_kps=prev_kps)
    kps = detector.detect(image[rs:re, cs:ce], None)
    kps, des = detector.compute(image, kps)
    ft_data = FeatureTrackingData(cam_idx, image, kps, des)
    return ft_data

  def _detect_multicam(self, camera_images):
    det_data = {}
    for cam_idx in self.cam_indices:
      image = camera_images[cam_idx]
      prev_kps = self._get_keypoints(cam_idx)
      det_data[cam_idx] = self._detect(cam_idx, image, prev_kps)
    return det_data

  def _match(self, data_i, data_j, sort_matches=True):
    # Match
    (cam_i, img_i, kps_i, des_i, _) = data_i.get_all()
    (cam_j, img_j, kps_j, des_j, _) = data_j.get_all()
    matches = self.matcher.match(des_i, des_j)
    if sort_matches:
      matches = sorted(matches, key=lambda x: x.distance)

    # Form matched keypoints and desciptor data structures
    matches_new = []
    kps_i_new = []
    kps_j_new = []
    des_i_new = np.zeros((len(matches), des_i.shape[1]), dtype=np.uint8)
    des_j_new = np.zeros((len(matches), des_j.shape[1]), dtype=np.uint8)

    row_idx = 0
    counter_src = {}
    counter_dst = {}
    for match in matches:
      kps_i_new.append(kps_i[match.queryIdx])
      kps_j_new.append(kps_j[match.trainIdx])
      des_i_new[row_idx, :] = des_i[match.queryIdx, :]
      des_j_new[row_idx, :] = des_j[match.trainIdx, :]
      matches_new.append(cv2.DMatch(row_idx, row_idx, match.distance))
      row_idx += 1

    # Form feature tracking data
    data_i = FeatureTrackingData(cam_i, img_i, kps_i_new, des_i_new)
    data_j = FeatureTrackingData(cam_j, img_j, kps_j_new, des_j_new)

    return (data_i, data_j, matches_new)

  def _detect_overlaps(self, camera_images):
    # Detect features for each camera
    det_data = self._detect_multicam(camera_images)

    # Add features
    for cam_i, cam_j in self.cam_overlaps:
      # Match keypoints and descriptors
      fdi = det_data[cam_i]
      fdj = det_data[cam_j]
      (data_i, data_j, matches) = self._match(fdi, fdj)

      # Add to camera data
      feature_ids = self._form_feature_ids(len(matches))
      data_i.feature_ids = feature_ids
      data_j.feature_ids = feature_ids
      self.cam_data[cam_i] = data_i
      self.cam_data[cam_j] = data_j

  def _detect_nonoverlaps(self, camera_images):
    # Detect features for each camera
    det_data = self._detect_multicam(camera_images)

    # Add features
    for cam_idx in det_data.keys():
      data = det_data[cam_idx]
      nb_features = len(data.keypoints)
      data.feature_ids = self._form_feature_ids(nb_features)
      self.cam_data[cam_idx] = data

  def _initialize(self, camera_images):
    # Detect new features
    if self.mode == "DEFAULT":
      self._detect_overlaps(camera_images)
      self._detect_nonoverlaps(camera_images)
    elif self.mode == "OVERLAPS_ONLY":
      self._detect_overlaps(camera_images)
    elif self.mode == "NON_OVERLAPS_ONLY":
      self._detect_nonoverlaps(camera_images)
    else:
      raise RuntimeError("Invalid FeatureTracker mode [%s]!" % self.mode)

    # Keep camera images
    self.prev_camera_images = camera_images

    # Draw visualization image
    viz = []
    for cam_idx in self.cam_indices:
      img = self.prev_camera_images[cam_idx]
      data = self.cam_data[cam_idx]
      viz.append(cv2.drawKeypoints(img, data.keypoints, None))

    return cv2.hconcat(viz)

  def _track_features(self, ts, camera_images):
    # Track features through time
    viz = []
    for cam_idx in self.cam_indices:
      # Previous image, keypoints and descriptors
      img_km1 = self.prev_camera_images[cam_idx]
      data_km1 = self.cam_data[cam_idx]

      # Current image, keypoints and descriptors
      img_k = camera_images[cam_idx]
      data_k = self._detect(cam_idx, img_k)

      # Match keypoints from past to current
      (data_km1, data_k, matches) = self._match(data_km1, data_k)
      (_, img_km1, kps_km1, _, _) = data_km1.get_all()
      (_, img_k, kps_k, _, _) = data_k.get_all()
      viz.append(cv2.drawMatches(img_km1, kps_km1, img_k, kps_k, matches, None))

    # Form visualization image
    return cv2.vconcat(viz)

  def update(self, ts, camera_images):
    # Track features
    viz = None
    if self.frame_idx == 0:
      viz = self._initialize(camera_images)
    else:
      viz = self._track_features(ts, camera_images)

    # Update
    self.frame_idx += 1

    return viz


###############################################################################
# CALIBRATION
###############################################################################

@dataclass
class CalibTarget:
  """ Calibration Target """
  nb_rows: int = 0
  nb_cols: int = 0
  tag_size: float = 0.0
  tag_spacing: float = 0.0


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
  for i in range(len(x_range)):
    for j in range(len(y_range)):
      for k in range(len(z_range)):
        x = x_range[i]
        y = y_range[j]
        z = z_range[k]
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

  angle_range = np.deg2rad([-20.0, 20.0])
  x_range = [-0.5, 0.5]
  y_range = [-0.5, 0.5]
  z_range = [0.5, 0.7]

  # For each position create a camera pose that "looks at" the AprilGrid
  # center in the target frame, T_TC.
  poses = []
  for i in range(nb_poses):
    # Generate random pose
    x = numpy.random.uniform(x_range[0], x_range[1])
    y = numpy.random.uniform(y_range[0], y_range[1])
    z = numpy.random.uniform(z_range[0], z_range[1])
    r_TC = calib_center + np.array([x, y, z])
    T_TC = lookat(r_TC, calib_center)

    # Perturb the pose a little so it doesn't look at the center directly
    yaw = numpy.random.uniform(angle_range)
    pitch = numpy.random.uniform(angle_range)
    roll = numpy.random.uniform(angle_range)
    C_perturb = euler321(yaw, pitch, roll)
    r_perturb = zeros((3, 1))
    T_perturb = tf(C_perturb, r_perturb)

    poses.append(T_perturb * T_TC)


class AprilGrid:
  """ AprilGrid """
  def __init__(self, tag_rows=6, tag_cols=6, tag_size=0.088, tag_spacing=0.3):
    self.tag_rows = tag_rows
    self.tag_cols = tag_cols
    self.tag_sizse = tag_size
    self.tag_spacing = tag_spacing
    self.object_points = self.object_points()
    self.keypoints = {}

  def grid_index(self, tag_id):
    assert tag_id < (self.tag_rows * self.tag_cols) and id >= 0
    i = floor(tag_id / self.tag_cols)
    j = floor(rem(tag_id, self.tag_cols))
    return (i, j)

  def object_points(self):
    object_points = []

    nb_tags = self.tag_rows * self.tag_cols
    for tag_id in range(nb_tags - 1):
      # Calculate the AprilGrid index using tag id
      [i, j] = self.grid_index(grid, tag_id)

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
      object_points.append(tag_points)

    return object_points


################################################################################
# DATASET
################################################################################

# EUROC ########################################################################


def load_euroc_dataset(data_path):
  """ Load EuRoC Dataset """
  # Form sensor paths
  imu0_path = os.path.join(data_path, 'mav0', 'imu0', 'data')
  cam0_path = os.path.join(data_path, 'mav0', 'cam0', 'data')
  cam1_path = os.path.join(data_path, 'mav0', 'cam1', 'data')

  # Data
  timestamps = []
  cam0_data = {}
  cam1_data = {}

  # Get camera0 data
  cam0_image_paths = sorted(glob.glob(os.path.join(cam0_path, '*.png')))
  for img_file in cam0_image_paths:
    ts_str, file_ext = os.path.basename(img_file).split('.')
    ts = int(ts_str)
    cam0_data[ts] = img_file
    timestamps.append(ts)

  # Get camera1 data
  cam1_image_paths = sorted(glob.glob(os.path.join(cam1_path, '*.png')))
  for img_file in cam1_image_paths:
    ts_str, file_ext = os.path.basename(img_file).split('.')
    ts = int(ts_str)
    cam1_data[ts] = img_file
    timestamps.append(ts)

  # Form dataset
  dataset = {}
  dataset['timestamps'] = sorted(set(timestamps))
  dataset['cam0'] = cam0_data
  dataset['cam1'] = cam1_data

  return dataset


###############################################################################
# SIMULATION
###############################################################################


@dataclass
class CameraEvent:
  ts: int
  cam_idx: int
  measurements: np.array

@dataclass
class ImuEvent:
  ts: int
  imu_idx: int
  acc: np.array
  gyr: np.array


def create_3d_features(x_bounds, y_bounds, z_bounds, nb_features):
  """ Create 3D features randomly """
  features = zeros((nb_features, 3))
  for i in range(nb_features):
    features[i, 0] = randf(x_bounds)
    features[i, 1] = randf(y_bounds)
    features[i, 2] = randf(z_bounds)
  return features


def create_3d_features_perimeter(origin, dim, nb_features):
  """ Create 3D features in a square """
  assert len(origin) == 3
  assert len(dim) == 3
  assert nb_features > 0

  # Dimension of the outskirt
  w = dim[0]
  l = dim[1]
  h = dim[2]

  # Features per side
  nb_fps = nb_features / 4.0

  # Features in the east side
  x_bounds = [origin[0] - w, origin[0] + w]
  y_bounds = [origin[1] + l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  east_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the north side
  x_bounds = [origin[0] + w, origin[0] + w]
  y_bounds = [origin[1] - l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  north_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the west side
  x_bounds = [origin[0] - w, origin[0] + w]
  y_bounds = [origin[1] - l, origin[1] - l]
  z_bounds = [origin[2] - h, origin[2] + h]
  west_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Features in the south side
  x_bounds = [origin[0] - w, origin[0] - w]
  y_bounds = [origin[1] - l, origin[1] + l]
  z_bounds = [origin[2] - h, origin[2] + h]
  south_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps)

  # Stack features and return
  features = np.array(
      [east_features, north_features, west_features, south_features])


def sim_vo_circle(circle_r, velocity, **kwargs):
  """ Simulate a camera going around in a circle """
  C_BC0 = euler321(deg2rad([-90.0, 0.0, -90.0]))
  r_BC0 = [0.01, 0.01, 0.05]
  T_BC0 = tf(C_BC0, r_BC0)
  nb_features = 1000

  # cam0
  cam_idx = 0
  image_width = 640
  image_height = 480
  resolution = [image_width, image_height]
  fov = 90.0
  fx = focal_length(image_width, fov)
  fy = focal_length(image_width, fov)
  cx = image_width / 2
  cy = image_height / 2
  proj_params = [fx, fy, cx, cy]
  dist_params = [-0.01, 0.01, 1e-4, 1e-4]
  cam0 = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params)

  # Simulate features
  origin = [0, 0, 0]
  dim = [circle_r * 2, circle_r * 2, circle_r * 1.5]
  features = create_3d_features_perimeter(origin, dim, nb_features)

  # Simulate camera
  cam_rate = 20.0
  circle_dist = 2.0 * pi * circle_r
  time_taken = circle_dist / velocity

  dt = 1.0 / cam_rate
  w = -2.0 * pi * (1.0 / time_taken)
  time = 0.0
  theta = pi
  yaw = pi / 2.0

  cam_time = []
  cam_poses = {}
  cam_pos = []
  cam_quat = []
  cam_att = []
  z_data = {}
  p_data = {}

  # Simulate camera
  idx = 1
  while (time <= time_taken):
    # Body pose
    rx = circle_r * cos(theta)
    ry = circle_r * sin(theta)
    rz = 0.0
    r_WB = [rx, ry, rz]
    rpy_WB = [0.0, 0.0, yaw]
    C_WB = euler321(rpy_WB)
    T_WB = tf(C_WB, r_WB)

    # Camera pose
    T_WC0 = T_WB * T_BC0
    [z_data, p_data] = camera_measurements(cam0, T_WC0, features.T)
    cam_time.append(time)
    cam_poses[time] = T_WC0
    cam_pos.append(tf_trans(T_WC0))
    cam_quat.append(tf_quat(T_WC0))
    cam_att.append(quat2euler(tf_quat(T_WC0)))
    cam_z_data.append(z_data)
    cam_p_data.append(p_data)

    # Update
    idx += 1
    theta += w * dt
    yaw += w * dt
    time += dt

  # # Form timeline
  # timeline = []
  # for k in range(cam_time):
  #   event = {}
  #   event.ts = cam_time(k)
  #   event.cam_pose = cam_poses{k}
  #   event.cam_z_data = cam_z_data{k}
  #   event.cam_p_data = cam_p_data{k}
  #   timeline.append(event)

  # # Simulation data
  # sim_data = {}
  # sim_data.timeline = timeline
  # # -- Features
  # sim_data.nb_features = nb_features
  # sim_data.features = features
  # # -- Camera
  # sim_data.T_BC0 = T_BC0
  # sim_data.cam0 = cam0
  # sim_data.cam_time = cam_time
  # sim_data.cam_poses = cam_poses
  # sim_data.cam_pos = cam_pos
  # sim_data.cam_quat = cam_quat
  # sim_data.cam_att = cam_att
  # sim_data.cam_z_data = cam_z_data
  # sim_data.cam_p_data = cam_p_data


def sim_imu_circle(circle_r, velocity):
  """ Simulate an IMU going around in a circle """
  imu_rate = 200.0
  circle_dist = 2.0 * pi * circle_r
  time_taken = circle_dist / velocity
  g = np.array([0.0, 0.0, 9.81])
  print("Simulating ideal IMU measurements ...")
  print("imu_rate: %f" % imu_rate)
  print("circle_r: %f" % circle_r)
  print("circle_dist: %f" % circle_dist)
  print("time_taken: %f" % time_taken)

  dt = 1.0 / imu_rate
  w = -2.0 * pi * (1.0 / time_taken)
  t = 0

  theta = pi
  yaw = pi / 2.0

  imu_poses = []
  imu_pos = []
  imu_quat = []
  imu_att = []
  imu_vel = []

  imu_time = []
  imu_acc = []
  imu_gyr = []

  idx = 1
  while (t <= time_taken):
    # IMU pose
    rx = circle_r * cos(theta)
    ry = circle_r * sin(theta)
    rz = 0.0
    r_WS = np.array([rx, ry, rz])
    rpy_WS = np.array([0.0, 0.0, yaw])
    C_WS = euler321(rpy_WS)
    T_WS = tf(C_WS, r_WS)

    # IMU velocity
    vx = -circle_r * w * sin(theta)
    vy = circle_r * w * cos(theta)
    vz = 0.0
    v_WS = np.array([vx, vy, vz])

    # IMU acceleration
    ax = -circle_r * w * w * cos(theta)
    ay = -circle_r * w * w * sin(theta)
    az = 0.0
    a_WS = np.array([ax, ay, az])

    # IMU angular velocity
    wx = 0.0
    wy = 0.0
    wz = w
    w_WS = np.array([wx, wy, wz])

    # IMU measurements
    acc = C_WS.T @ (a_WS + g)
    gyr = C_WS.T @ w_WS

    # Update
    imu_poses.append(T_WS)
    imu_pos.append(tf_trans(T_WS))
    imu_quat.append(tf_quat(T_WS))
    imu_att.append(quat2euler(tf_quat(T_WS)))
    imu_vel.append(v_WS)

    imu_time.append(t)
    imu_acc.append(acc)
    imu_gyr.append(gyr)

    idx += 1
    theta += w * dt
    yaw += w * dt
    t += dt

  # sim_data = {}
  # sim_data.imu_poses = imu_poses
  # sim_data.imu_pos = imu_pos
  # sim_data.imu_quat = imu_quat
  # sim_data.imu_att = imu_att
  # sim_data.imu_vel = imu_vel
  # sim_data.imu_time = imu_time
  # sim_data.imu_acc = imu_acc
  # sim_data.imu_gyr = imu_gyr


# def camera_measurements(camera, T_WC, points_W):
#   assert(T_WC.shape [4, 4]);
#   assert(points_W.shape[0] == 3);
#   assert(points_W.shape[1] > 0);
#
#   # Form projection matrix
#   T_CW = tf_inv(T_WC)
#   C_CW = tf_rot(T_CW)
#   r_CW = tf_trans(T_CW)
#   K = pinhole_K(camera.param(1:4))
#   P = K @ np.array([C_CW, r_CW])

#   # Setup
#   image_width = camera.resolution[0];
#   image_height = camera.resolution[1];
#   nb_points = columns(points_W);
#
#   # Check points
#   z = [];
#   point_ids = [];
#
#   for i = 1:nb_points
#     # Project point to image plane
#     hp_W = homogeneous(points_W(:, i));
#     x = P * hp_W;
#
#     # Check to see if point is infront of camera
#     if x[2] < 1e-4
#       continue;
#     endif
#
#     # Normalize projected ray
#     x[0] = x[0] / x[2];
#     x[1] = x[1] / x[2];
#     x[2] = x[2] / x[2];
#     z_hat = [x[0]; x[1]];
#
#     # Check to see if ray is within image plane
#     x_ok = (x[0] < image_width) and (x[0] > 0.0);
#     y_ok = (x[1] < image_height) and (x[1] > 0.0);
#     if x_ok && y_ok
#       z = [z, z_hat];
#       point_ids = [point_ids, i];
#     endif
#   endfor
#   assert(columns(z) == length(point_ids));
# endfunction

###############################################################################
#                               UNITTESTS
###############################################################################

import unittest

# LINEAR ALGEBRA ##############################################################


class TestLinearAlgebra(unittest.TestCase):
  def test_normalize(self):
    x = np.array([1.0, 2.0, 3.0])
    n = norm(x)
    x_prime = normalize(x)
    self.assertTrue(isclose(norm(x_prime), 1.0))

  def test_skew(self):
    x = np.array([1.0, 2.0, 3.0])
    S = np.array([[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]])
    self.assertTrue(matrix_equal(S, skew(x)))

  def test_skew_inv(self):
    x = np.array([1.0, 2.0, 3.0])
    S = np.array([[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]])
    self.assertTrue(matrix_equal(x, skew_inv(S)))

  def test_matrix_equal(self):
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
  def test_Exp_Log(self):
    pass


# TRANSFORM ###################################################################


class TestTransform(unittest.TestCase):
  def test_homogeneous(self):
    p = np.array([1.0, 2.0, 3.0])
    hp = homogeneous(p)
    self.assertTrue(hp[0] == 1.0)
    self.assertTrue(hp[1] == 2.0)
    self.assertTrue(hp[2] == 3.0)
    self.assertTrue(len(hp) == 4)

  def test_dehomogeneous(self):
    hp = np.array([1.0, 2.0, 3.0, 1.0])
    p = dehomogeneous(hp)
    self.assertTrue(p[0] == 1.0)
    self.assertTrue(p[1] == 2.0)
    self.assertTrue(p[2] == 3.0)
    self.assertTrue(len(p) == 3)

  def test_rotx(self):
    x = np.array([0.0, 1.0, 0.0])
    C = rotx(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 0.0, 1.0]))

  def test_roty(self):
    x = np.array([1.0, 0.0, 0.0])
    C = roty(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 0.0, -1.0]))

  def test_rotz(self):
    x = np.array([1.0, 0.0, 0.0])
    C = rotz(deg2rad(90.0))
    x_prime = C @ x
    self.assertTrue(np.allclose(x_prime, [0.0, 1.0, 0.0]))

  def test_aa2quat(self):
    pass

  def test_rvec2rot(self):
    pass

  def test_vecs2axisangle(self):
    pass

  def test_euler321(self):
    C = euler321(0.0, 0.0, 0.0)
    self.assertTrue(np.array_equal(C, eye(3)))

  def test_euler2quat_and_quat2euler(self):
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
    pass

  def test_rot2euler(self):
    pass

  def test_rot2quat(self):
    pass

  def test_quat_norm(self):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    self.assertTrue(isclose(quat_norm(q), 1.0))

  def test_quat_normalize(self):
    q = np.array([1.0, 0.1, 0.2, 0.3])
    q = quat_normalize(q)
    self.assertTrue(isclose(quat_norm(q), 1.0))

  def test_quat_conj(self):
    pass

  def test_quat_inv(self):
    pass

  def test_quat_mul(self):
    p = euler2quat(deg2rad(3.0), deg2rad(2.0), deg2rad(1.0))
    q = euler2quat(deg2rad(1.0), deg2rad(2.0), deg2rad(3.0))
    r = quat_mul(p, q)

  def test_quat_omega(self):
    pass

  def test_tf(self):
    pass
    # pose = [1.0; 0.0; 0.0; 0.0; 1.0; 2.0; 3.0];
    # T = tf(pose);
    # self.assertTrue(isequal(T(1:3, 1:3), eye(3)) == 1);
    # self.assertTrue(isequal(T(1:3, 4), [1; 2; 3]) == 1);

    # C = [[1.0, 0.0, 0.0];
    #     [0.0, 2.0, 0.0];
    #     [0.0, 0.0, 3.0]];
    # r = [1.0; 2.0; 3.0];
    # T = tf(C, r);
    # self.assertTrue(isequal(T(1:3, 1:3), C) == 1);
    # self.assertTrue(isequal(T(1:3, 4), r) == 1);


# CV ##########################################################################


class TestCV(unittest.TestCase):
  def setUp(self):
    # Camera
    img_w = 640
    img_h = 480
    fx = 320.0
    fy = 320.0
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

  def test_poinhole_K(self):
    fx = 1.0
    fy = 2.0
    cx = 3.0
    cy = 4.0
    proj_params = [fx, fy, cx, cy]
    K = pinhole_K(proj_params)
    expected = np.array([[1.0, 0.0, 3.0], [0.0, 2.0, 4.0], [0.0, 0.0, 1.0]])

    self.assertTrue(np.array_equal(K, expected))

  def test_pinhole_project(self):
    z = pinhole_project(self.proj_params, self.p_C)
    self.assertTrue(isclose(z[0], 320.0))
    self.assertTrue(isclose(z[1], 240.0))

  def test_pinhole_params_jacobian(self):
    # Pinhole params jacobian
    fx, fy, cx, cy = self.proj_params
    z = np.array([fx * self.x[0] + cx, fy * self.x[1] + cy])
    J = pinhole_params_jacobian(self.proj_params, self.x)

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


class TestFeatureTracking(unittest.TestCase):
  def setUp(self):
    # Load test data
    data_path = '/data/euroc/raw/V1_01'
    self.dataset = load_euroc_dataset(data_path)

    # Setup test images
    ts = self.dataset['timestamps'][0]
    self.img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    self.img1 = cv2.imread(self.dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    # Setup feature tracker
    self.feature_tracker = FeatureTracker()
    self.feature_tracker.add_camera('cam0')
    self.feature_tracker.add_camera('cam1')
    self.feature_tracker.add_overlap('cam0', 'cam1')

  def test_feature_grid_cell_index(self):
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
    grid_rows = 4
    grid_cols = 4
    image_shape = (320, 280)
    keypoints = [[0, 0], [320, 0], [0, 280], [320, 280]]
    grid = FeatureGrid(grid_rows, grid_cols, image_shape, keypoints)

    self.assertEqual(grid.count(0), 1)
    self.assertEqual(grid.count(3), 1)
    self.assertEqual(grid.count(12), 1)
    self.assertEqual(grid.count(15), 1)

  def test_grid_detect(self):
    debug = False
    feature = cv2.ORB_create(nfeatures=100)
    kps, des = grid_detect(feature, self.img0, debug=debug)
    self.assertTrue(len(kps) > 0)
    self.assertEqual(des.shape[0], len(kps))

  def test_optflow_track(self):
    debug = False

    # Detect
    feature = cv2.ORB_create(nfeatures=100)
    kps, des = grid_detect(feature, self.img0)

    # Track
    pts_i = np.array([kp.pt for kp in kps], dtype=np.float32)
    track_results = optflow_track(self.img0, self.img1, pts_i, debug=debug)
    (pts_i, pts_j, inliers) = track_results

    self.assertTrue(len(pts_i) == len(pts_j))


class TestFeatureTracker(unittest.TestCase):
  def setUp(self):
    # Load test data
    data_path = '/data/euroc/raw/V1_01'
    self.dataset = load_euroc_dataset(data_path)

    # Setup test images
    ts = self.dataset['timestamps'][0]
    self.img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    self.img1 = cv2.imread(self.dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    # Setup feature tracker
    self.feature_tracker = FeatureTracker()
    self.feature_tracker.add_camera('cam0')
    self.feature_tracker.add_camera('cam1')
    self.feature_tracker.add_overlap('cam0', 'cam1')

  def test_detect(self):
    # Load and detect features from single image
    ts = self.dataset['timestamps'][0]
    img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    ft_data = self.feature_tracker._detect(img0)

  def test_detect_multicam(self):
    # Feed camera images to feature tracker
    camera_images = {}
    camera_images['cam0'] = self.img0
    camera_images['cam1'] = self.img1
    self.feature_tracker._detect_multicam(camera_images)

  def test_match(self):
    debug = True

    # Load images
    camera_images = {}
    camera_images['cam0'] = self.img0
    camera_images['cam1'] = self.img1

    # Detect
    fd0 = self.feature_tracker._detect('cam0', self.img0)
    fd1 = self.feature_tracker._detect('cam1', self.img1)
    (data_i, data_j, matches) = self.feature_tracker._match(fd0, fd1)
    self.assertEqual(len(data_i.keypoints), len(data_j.keypoints))

    # Visualize
    if debug:
      kps0 = data_i.keypoints
      kps1 = data_j.keypoints
      viz = cv2.drawMatches(self.img0, kps0, self.img1, kps1, matches, None)
      cv2.imshow('viz', viz)
      cv2.waitKey(0)

  def test_detect_overlaps(self):
    debug = True

    # Feed camera images to feature tracker
    camera_images = {}
    camera_images['cam0'] = self.img0
    camera_images['cam1'] = self.img1
    self.feature_tracker._detect_overlaps(camera_images)

    # Visualize
    for cam_i, cam_j in self.feature_tracker.cam_overlaps:
      img_i = camera_images[cam_i]
      img_j = camera_images[cam_j]
      data_i = self.feature_tracker.cam_data[cam_i]
      data_j = self.feature_tracker.cam_data[cam_j]
      kps_i = data_i.keypoints
      kps_j = data_j.keypoints
      viz = draw_matches(img_i, img_j, kps_i, kps_j)
      cv2.imshow('viz', viz)
      cv2.waitKey(0)

  def test_detect_nonoverlaps(self):
    pass

  def test_initialize(self):
    ts = self.dataset['timestamps'][0]
    img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(self.dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    camera_images = {}
    camera_images['cam0'] = img0
    camera_images['cam1'] = img1
    self.feature_tracker._initialize(camera_images)

  def test_run(self):
    for ts in self.dataset['timestamps']:
      print(ts)
      # Load images
      img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
      img1 = cv2.imread(self.dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

      # Feed camera images to feature tracker
      camera_images = {}
      camera_images['cam0'] = img0
      camera_images['cam1'] = img1
      viz = self.feature_tracker.update(ts, camera_images)

      # Visualize
      sys.stdout.flush()
      cv2.imshow('viz', viz)
      # if cv2.waitKey(0) == ord('q'):
      #   break
      if cv2.waitKey(1) == ord('q'):
        break


# CALIBRATION #################################################################


class TestCalibration(unittest.TestCase):
  def test_dummy(self):
    pass


# DATASET  ####################################################################


class TestEuoc(unittest.TestCase):
  def test_load(self):
    data_path = '/data/euroc/raw/V1_01'
    self.dataset = load_euroc_dataset(data_path)


# SIMULATION  #################################################################


class TestSimulation(unittest.TestCase):
  def test_dummy(self):
    pass


if __name__ == '__main__':
  unittest.main()
