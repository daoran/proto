import math

import numpy as np

###############################################################################
# MATHS
###############################################################################


def isclose(a, b, tol=1e-8):
  return math.isclose(a, b, abs_tol=tol)


###############################################################################
# LINEAR ALGEBRA
###############################################################################


def rows(x):
  return x.shape[0]


def columns(x):
  return x.shape[1]


def deg2rad(x):
  return np.deg2rad(x)


def rad2deg(x):
  return np.rad2deg(x)


def ones(x):
  return np.ones(x)


def zeros(x):
  return np.zeros(x)


def eye(x):
  return np.eye(x)


def diag(x):
  return np.diagonal(x)


def norm(v):
  return np.linalg.norm(v)


def normalize(v):
  norm = np.linalg.norm(v)
  if norm == 0:
    return v
  return v / norm


def inv(v):
  return np.linalg.inv(v)


def pinv(v):
  return np.linalg.pinv(v)


def rank(A):
  return np.linalg.matrix_rank(A)


def fullrank(A):
  return rank(A) == A.shape[0]


def skew(vec):
  assert (vec.shape == (3,) or vec.shape == (3, 1))
  return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def skew_inv(A):
  assert (A.shape == (3, 3))
  return np.array([A[2, 1], A(0, 2), A(1, 0)])


def eig(v):
  return np.linalg.eig(v)


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


def schurs_complement(H, g, m, r, precond=False):
  assert (H.shape[0] == (m + r))

  # H = [Hmm, Hmr
  #      Hrm, Hrr];
  Hmm = H[0:m, 0:m]
  Hmr = H[0:m, m:]
  Hrm = Hmr.transpose()
  Hrr = H[m:, m:]

  # g = [gmm, grr]
  gmm = g[1:]
  grr = g[m:]

  # Precondition Hmm
  if (precond):
    Hmm = 0.5 * (Hmm + Hmm.transpose())

  # Invert Hmm
  assert (rank(Hmm) == Hmm.shape[0])
  (w, V) = eig(Hmm)
  W_inv = diag(1.0 / w)
  Hmm_inv = V * W_inv * V.transpose()

  # Schurs complement
  H_marg = Hrr - Hrm * Hmm_inv * Hmr
  g_marg = grr - Hrm * Hmm_inv * gmm

  return (H_marg, g_marg)


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


def axisangle2quat(axis, angle):
  ax, ay, az = axis
  qw = math.cos(angle / 2.0)
  qx = ax * math.sin(angle / 2.0)
  qy = ay * math.sin(angle / 2.0)
  qz = az * math.sin(angle / 2.0)
  return np.array([qw, qx, qy, qz])


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

  cpsi = math.cos(psi)
  spsi = math.sin(psi)
  ctheta = math.cos(theta)
  stheta = math.sin(theta)
  cphi = math.cos(phi)
  sphi = math.sin(phi)

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


# QUATERNION ##################################################################


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


# TF ##########################################################################


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


###############################################################################
# CV
###############################################################################


def lookat(cam_pos, target_pos, up_axis=[0.0, -1.0, 0.0]):
  assert (cam_pos.shape == (3,) or cam_pos.shape == (3, 1))
  assert (target_pos.shape == (3,) or target_pos.shape == (3, 1))
  assert (up_axis.shape == (3,) or up_axis.shape == (3, 1))

  # Note: If we were using OpenGL the cam_dir would be the opposite direction,
  # since in OpenGL the camera forward is -z. In robotics however our camera is
  # +z forward.
  cam_dir = normalize((target_pos - cam_pos))
  cam_right = normalize(cross(up_axis, cam_dir))
  cam_up = cross(cam_dir, cam_right)

  A = zeros(4, 4)
  A[0, :] = [cam_right(1), cam_right(2), cam_right(3), 0.0]
  A[1, :] = [cam_up(1), cam_up(2), cam_up(3), 0.0]
  A[2, :] = [cam_dir(1), cam_dir(2), cam_dir(3), 0.0]
  A[3, :] = [0.0, 0.0, 0.0, 1.0]

  B = zeros(4, 4)
  B[0, :] = [1.0, 0.0, 0.0, -cam_pos(1)]
  B[1, :] = [0.0, 1.0, 0.0, -cam_pos(2)]
  B[2, :] = [0.0, 0.0, 1.0, -cam_pos(3)]
  B[3, :] = [0.0, 0.0, 0.0, 1.0]

  T_camera_target = A @ B
  T_target_camera = inv(T_camera_target)
  return T_target_camera


def linear_triangulation(P_i, P_j, z_i, z_j):
  # Linear triangulation
  # This function is used to triangulate a single 3D point observed by two
  # camera frames (be it in time with the same camera, or two different cameras
  # with known extrinsics)

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
  A = zeros(4, 1)
  # A = [y * P3T - P2T;
  #      x * P3T - P1T;
  #      y_dash * P3T_dash - P2T_dash;
  #      x_dash * P3T_dash - P1T_dash];

  # Use SVD to solve AX = 0
  (_, _, V) = svd(A.transpose() * A)
  hp = V[:, 3]  # Get the best result from SVD (last column of V)
  hp = hp / hp(3)  # Normalize the homogeneous 3D point
  p = hp[0:3]  # Return only the first three components (x, y, z)


# PINHOLE #####################################################################


def pinhole_project(proj_params, T_WC, p_W):
  assert (len(proj_params) == 4)
  assert (T_WC.shape == (4, 4))
  assert (p_W.shape == (3, 1) or p_W.shape == (3,))

  T_CW = tf_inv(T_WC)
  p_C = tf_point(T_CW, p_W)

  fx, fy, cx, cy = proj_params

  x = np.array([p_C[0] / p_C[2], p_C(2) / p_C[2]])  # Project
  z = np.array([fx * x[0] + cx, fy * x[1] + cy])
  # Scale and center
  return z


def pinhole_K(params):
  fx, fy, cx, cy = params
  return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def pinhole_params_jacobian(proj_params, x):
  return np.array([[x[0], 0.0, 1.0, 0.0], [0.0, x[1], 0.0, 1.0]])


def pinhole_point_jacobian(proj_params):
  fx, fy, _, _ = proj_params
  return np.array([[fx, 0.0], [0.0, fy]])


# RADTAN4 #####################################################################


def radtan4_distort(dist_params, p):
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

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
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

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
    if (err.transpose() * err) < 1e-15:
      break

  return p


def radtan4_params_jacobian(dist_params, p):
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

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
  J_params = zeros(2, 4)
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
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  # Distort
  x, y = p
  r = math.sqrt(x * x + y * y)
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
  thd = math.sqrt(p(0) * p(0) + p(1) * p(1))

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
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  # Jacobian
  x, y = p
  r = math.sqrt(x**2 + y**2)
  th = atan(r)

  J_params = zeros(2, 4)
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
  assert (len(dist_params) == 4)
  assert (len(p) == 2)

  # Distortion parameters
  k1, k2, k3, k4 = dist_params

  # Jacobian
  x, y = p
  r = math.sqrt(x**2 + y**2)

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

  J_point = zeros(2, 2)
  J_point[0, 0] = s + x * s_r * r_x
  J_point[0, 1] = x * s_r * r_y
  J_point[1, 0] = y * s_r * r_x
  J_point[1, 1] = s + y * s_r * r_y

  return J_point


###############################################################################
# CALIBRATION
###############################################################################


class CalibTarget:
  def __init__(self):
    self.nb_rows = 0.0
    self.nb_cols = 0.0
    self.tag_size = 0.0
    self.tag_spacing = 0.0


def calib_generate_poses(calib_target):
  # Settings
  calib_width = (calib_target.nb_cols - 1.0) * calib_target.tag_size
  calib_height = (calib_target.nb_rows - 1.0) * calib_target.tag_size
  calib_center = np.array([calib_width / 2.0, calib_height / 2.0, 0.0])

  # Pose settings
  x_range = np.linspace(-0.3, 0.3, 5)
  y_range = np.linspace(-0.3, 0.3, 5)
  z_range = np.linspace(0.2, 0.5, 5)

  # Generate camera positions infront of the AprilGrid target r_TC
  cam_pos = zeros(3, len(x_range) * len(y_range) * len(z_range))
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
    r_perturb = zeros(3, 1)
    T_perturb = tf(C_perturb, r_perturb)

    poses.append(T_perturb * T_TC)


###############################################################################
# SIMULATION
###############################################################################

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
#   image_width = camera.resolution(1);
#   image_height = camera.resolution(2);
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
#     if x(3) < 1e-4
#       continue;
#     endif
#
#     # Normalize projected ray
#     x(1) = x(1) / x(3);
#     x(2) = x(2) / x(3);
#     x(3) = x(3) / x(3);
#     z_hat = [x(1); x(2)];
#
#     # Check to see if ray is within image plane
#     x_ok = (x(1) < image_width) && (x(1) > 0.0);
#     y_ok = (x(2) < image_height) && (x(2) > 0.0);
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


class LinearAlgebraTests(unittest.TestCase):
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

  def test_euler321(self):
    C = euler321(0.0, 0.0, 0.0)
    self.assertTrue(np.array_equal(C, eye(3)))


class TransformTests(unittest.TestCase):
  def test_quat_mul(self):
    p = euler2quat(deg2rad(3.0), deg2rad(2.0), deg2rad(1.0))
    q = euler2quat(deg2rad(1.0), deg2rad(2.0), deg2rad(3.0))
    r = quat_mul(p, q)

  def test_quat_norm(self):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    self.assertTrue(isclose(quat_norm(q), 1.0))

  def test_quat_normalize(self):
    q = np.array([1.0, 0.1, 0.2, 0.3])
    q = quat_normalize(q)
    self.assertTrue(isclose(quat_norm(q), 1.0))

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


if __name__ == '__main__':
  unittest.main()
