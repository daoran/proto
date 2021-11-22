import math
import numpy as np

###############################################################################
# LINEAR ALGEBRA
###############################################################################


def ones(x):
  return np.ones(x)


def zeros(x):
  return np.zeros(x)


def eye(x):
  return np.eye(x)


def norm(v):
  return np.linalg.norm(v)


def normalize(v):
  norm = np.linalg.norm(v)
  if norm == 0:
    return v
  return v / norm


def skew(vec):
  assert (vec.shape == (3,) or vec.shape == (3, 1))
  return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def skew_inv(A):
  assert (A.shape == (3, 3))
  return np.array([A[2, 1], A(0, 2), A(1, 0)])


def fwdsubs(L, b):
  """
  Solving a lower triangular system by forward-substitution
  Input matrix L is an n by n lower triangular matrix
  Input vector b is n by 1
  Output vector x is the solution to the linear system
  L x = b
  """
  assert (L.shape[1] == b.shape[0])
  n = b.shape[0]

  x = np.zeros(n, 1)
  for j in range(n):
    if L[j, j] == 0:
      error('Matrix is singular!')
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
  assert (U.shape[1] == b.shape[0])
  n = b.shape[0]

  x = np.zeros(n, 1)
  for j in range(n):
    if U[j, j] == 0:
      raise RuntimeError('Matrix is singular!')
    x[j] = b[j] / U(j, j)
    b[0:j] = b[0:j] - U[0:j, j] * x[j]


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
  assert (phi.shape == (3,) or phi.shape == (3, 1))
  if (phi < 1e-3):
    C = eye(3) + skew(phi)
    return C
  else:
    phi_norm = norm(phi)
    phi_skew = skew(phi)
    phi_skew_sq = phi_skew * phi_skew

    C = eye(3)
    C += (math.sin(phi_norm) / phi_norm) * phi_skew
    C += ((1 - math.cos(phi_norm)) / phi_norm ^ 2) * phi_skew_sq
    return C


def Log(C):
  assert (C.shape == (3, 3))
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
      magnitude = theta / (2.0 * math.sin(theta))
    else:
      # when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      # use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      # see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0
    rvec = magnitude * np.array([C21 - C12, C02 - C20, C10 - C01])


def Jr(theta):
  """
  Equation (8) in:
  Forster, Christian, et al. "IMU preintegration on manifold for efficient
  visual-inertial maximum-a-posteriori estimation." Georgia Institute of
  Technology, 2015.
  """
  theta_norm = norm(theta)
  theta_norm_sq = theta_norm * theta_norm
  theta_norm_cube = theta_norm_sq * theta_norm
  theta_skew = skew(theta)
  theta_skew_sq = theta_skew * theta_skew

  J = eye(3)
  J -= ((1 - math.cos(theta_norm)) / theta_norm_sq) * theta_skew
  J += (theta_norm - math.sin(theta_norm)) / (theta_norm_cube) * theta_skew_sq
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


###############################################################################
# TRANSFORM
###############################################################################


def axisangle2quat(axis, angle):
  ax, ay, az = axis
  qw = math.cos(angle / 2.0)
  qx = ax * math.sin(angle / 2.0)
  qy = ay * math.sin(angle / 2.0)
  qz = az * math.sin(angle / 2.0)
  return np.array([qw, qx, qy, qz])


def dehomogeneous(hp):
  return hp[0:2, :]


def homogeneous(p):
  return np.array([p, np.ones(1, p.shape[1])])


def rotx(theta):
  row0 = [0.0, 1.0, 0.0]
  row1 = [0.0, math.cos(theta), -math.sin(theta)]
  row2 = [0.0, math.sin(theta), math.cos(theta)]
  return np.array([row0, row1, row2])


def roty(theta):
  row0 = [cos(theta), 0.0, sin(theta)]
  row1 = [0.0, 1.0, 0.0]
  row2 = [-sin(theta), 0.0, cos(theta)]
  return np.array([row0, row1, row2])


def rotz(theta):
  row0 = [math.cos(theta), -math.sin(theta), 0.0]
  row1 = [math.sin(theta), math.cos(theta), 0.0]
  row2 = [0.0, 0.0, 1.0]
  return np.array([row0, row1, row2])


def euler2quat(yaw, pitch, roll):
  """
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  Page 166-167, "Euler Angles to Quaternion"
  """
  psi = yaw  # Yaw
  theta = pitch  # Pitch
  phi = roll  # Roll

  c_phi = math.cos(phi / 2.0)
  c_theta = math.cos(theta / 2.0)
  c_psi = math.cos(psi / 2.0)
  s_phi = math.sin(phi / 2.0)
  s_theta = math.sin(theta / 2.0)
  s_psi = math.sin(psi / 2.0)

  qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi
  qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi
  qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi
  qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi

  mag = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
  return np.array([qw / mag, qx / mag, qy / mag, qz / mag])


def euler321(yaw, pitch, roll):
  """
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  Page 85-86, "The Aerospace Sequence"
  """
  psi = yaw
  theta = pitch
  phi = roll

  C11 = cos(psi) * cos(theta)
  C21 = sin(psi) * cos(theta)
  C31 = -sin(theta)

  C12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
  C22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
  C32 = cos(theta) * sin(phi)

  C13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)
  C23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)
  C33 = cos(theta) * cos(phi)

  return np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])


def quat2euler(q):
  """
  Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  Princeton University Press, 1999. Print.
  Page 168, "Quaternion to Euler Angles"
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
  Blanco, Jose-Luis. "A tutorial on se (3) transformation parameterizations
  and on-manifold optimization." University of Malaga, Tech. Rep 3 (2010): 6.
  Page 18, Equation (2.20)
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
  assert C.shape == (3, 3)
  q = rot2quat(C)
  return quat2euler(q)


def rot2quat(C):
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
    S = math.sqrt(tr + 1.0) * 2
    # S=4*qw
    qw = 0.25 * S
    qx = (m21 - m12) / S
    qy = (m02 - m20) / S
    qz = (m10 - m01) / S
  elif ((m00 > m11) and (m00 > m22)):
    S = math.sqrt(1.0 + m00 - m11 - m22) * 2
    # S=4*qx
    qw = (m21 - m12) / S
    qx = 0.25 * S
    qy = (m01 + m10) / S
    qz = (m02 + m20) / S
  elif (m11 > m22):
    S = math.sqrt(1.0 + m11 - m00 - m22) * 2
    # S=4*qy
    qw = (m02 - m20) / S
    qx = (m01 + m10) / S
    qy = 0.25 * S
    qz = (m12 + m21) / S
  else:
    S = math.sqrt(1.0 + m22 - m00 - m11) * 2
    # S=4*qz
    qw = (m10 - m01) / S
    qx = (m02 + m20) / S
    qy = (m12 + m21) / S
    qz = 0.25 * S

  return quat_normalize(np.array([qw, qx, qy, qz]))


def rvec2rot(rvec):
  # If small rotation
  theta = math.sqrt(rvec @ rvec)  # = norm(rvec), but faster
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

  c = math.cos(theta)
  s = math.sin(theta)
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
  angle = math.acos(u.transpose() * v)
  ax = normalize(np.cross(u, v))
  return ax * angle


def quat_norm(q):
  qw, qx, qy, qz = q
  return math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)


def quat_normalize(q):
  n = quat_norm(q)
  qw, qx, qy, qz = q
  return np.array([qw / n, qx / n, qy / n, qz / n])


def quat_conj(q):
  qw, qx, qy, qz = q
  q_conj = np.array([qw, -qx, -qy, -qz])
  return q_conj


def quat_inv(q):
  return quat_conj(q)


def quat_left(q):
  qw, qx, qy, qz = q
  # yapf: disable
  L = np.array([qw, -qx, -qy, -qz,
                qx, qw, -qz, qy,
                qy, qz, qw, -qx,
                qz, -qy, qx, qw]).reshape((4, 4))
  # yapf: enable
  return L


def quat_right(q):
  qw, qx, qy, qz = q
  # yapf: disable
  R = np.array([qw, -qx, -qy, -qz,
                qx, qw, qz, -qy,
                qy, -qz, qw, qx,
                qz, qy, -qx, qw]).reshape((4, 4))
  # yapf: enable
  return R


def quat_lmul(p, q):
  assert len(p) == 4
  assert len(q) == 4
  lprod = quat_left(p)
  return lprod @ q


def quat_rmul(p, q):
  assert len(p) == 4
  assert len(q) == 4
  rprod = quat_right(q)
  return rprod @ p


def quat_mul(p, q):
  return quat_lmul(p, q)


def quat_omega(w):
  # Omega = [-skew(w), w;
  #          -transpose(w), 0.0];
  pass


def quat_delta(dalpha):
  half_norm = 0.5 * np.linalg.norm(dalpha)
  scalar = math.cos(half_norm)
  vector = math.sinc(half_norm) * 0.5 * dalpha
  dq = np.array([scalar, vector])  # (qw, qx, qy, qz)
  return dq


def quat_integrate(q_k, w, dt):
  """
  "Quaternion kinematics for the error-state Kalman filter" (2017)
  By Joan Sola
  [Section 4.6.1 Zeroth-order integration, p.47]
  """
  w_norm = np.linalg.norm(w)
  q_scalar = 0.0
  q_vec = np.array([0.0, 0.0, 0.0])

  if w_norm > 1e-5:
    q_scalar = math.cos(w_norm * dt * 0.5)
    q_vec = w / w_norm * math.sin(w_norm * dt * 0.5)
  else:
    q_scalar = 1.0
    q_vec = [0.0, 0.0, 0.0]

  q_kp1 = quat_mul(q_k, np.array([q_scalar, q_vec]))
  return q_kp1


def tf(rot, trans):
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
  return T[0:3, 0:3]


def tf_quat(T):
  return rot2quat(tf_rot(T))


def tf_inv(T):
  return np.linalg.inv(T)


def tf_point(T, p):
  assert p.shape == (3,) or p.shape == (3, 1)
  hpoint = np.array([p[0], p[1], p[2], 1.0])
  return (T @ hpoint)[0:3]


def tf_decompose(tf):
  C = tf_rot(tf)
  r = tf_trans(tf)
  return (C, r)


def tf_vector(T):
  rx, ry, rz = tf_trans(T)
  qw, qx, qy, qz = tf_quat(T)
  return np.array([rx, ry, rz, qw, qx, qy, qz])
