""" Robot Arm prototype """
#!/usr/bin/env python3
import numpy as np
from numpy import pi

from proto import hat
from proto import vee
from proto import aa_vec
from proto import aa_decomp
from proto import tf
from proto import tf_point
from proto import deg2rad
from proto import euler321
from proto import AprilGrid
from proto import focal_length
from proto import camera_params_setup
from proto import plot_tf
from proto import plot_set_axes_equal

import matplotlib.pylab as plt


def screw_to_se3(s):
  """ Screw axis to se(3)

  Let s be the screw axis:

    s = [w, v]

  This function turns

    s  -> [s]
    R6 -> se(3)

  Example Input:

    s = np.array([1, 2, 3, 4, 5, 6])

  Example Output:

    np.array([[ 0, -3,  2, 4],
              [ 3,  0, -1, 5],
              [-2,  1,  0, 6],
              [ 0,  0,  0, 0]])

  """
  w = s[0:3]
  v = s[3:]
  return np.block([[hat(w), v.reshape((3, 1))], [np.zeros((1, 4))]])


def se3_to_screw(se3mat):
  """ Convert se(3) matrix back to screw axis

  Example Input:

    np.array([[ 0, -3,  2, 4],
              [ 3,  0, -1, 5],
              [-2,  1,  0, 6],
              [ 0,  0,  0, 0]])

  Example Output:

    s = np.array([1, 2, 3, 4, 5, 6])

  """
  w = np.array([se3mat[2][1], se3mat[0][2], se3mat[1][0]])
  v = np.array([se3mat[0][3], se3mat[1][3], se3mat[2][3]])
  return np.array([*w, *v])


def so3_exp(so3mat, tol=1e-6):
  """ Computes the matrix exponential of a matrix in so(3)

  Example Input:
    so3mat = np.array([[ 0, -3,  2],
                       [ 3,  0, -1],
                       [-2,  1,  0]])
  Output:

    np.array([[-0.69492056,  0.71352099,  0.08929286],
              [-0.19200697, -0.30378504,  0.93319235],
              [ 0.69297817,  0.6313497 ,  0.34810748]])

  """
  aa = vee(so3mat)
  if np.linalg.norm(aa) < tol:
    return np.eye(3)

  _, theta = aa_decomp(aa)
  omgmat = so3mat / theta

  I3 = np.eye(3)
  s_theta = np.sin(theta)
  c_theta = np.cos(theta)

  return I3 + s_theta * omgmat + (1.0 - c_theta) * np.dot(omgmat, omgmat)


def so3_Exp(w):
  """ Exponential Map R3 to so3 """
  return so3_exp(hat(w))


def poe(screw_axis, joint_angle, tol=1e-6):
  """ Matrix exponential of se(3) matrix """
  s = screw_axis * joint_angle
  aa = s[0:3]  # Axis-angle (w * theta)
  v = s[3:]  # Linear velocity

  if np.linalg.norm(aa) < tol:
    return np.block([[np.eye(3), v.reshape((3, 1))], [0, 0, 0, 1]])

  se3mat = screw_to_se3(s)

  _, theta = aa_decomp(aa)
  w_skew = se3mat[0:3, 0:3] / theta
  w_skew_sq = w_skew @ w_skew

  I3 = np.eye(3)
  c_th = np.cos(theta)
  s_th = np.sin(theta)

  A = so3_exp(se3mat[0:3, 0:3])
  B = (I3 * theta + (1.0 - c_th) * w_skew + (theta - s_th) * w_skew_sq) @ v

  return np.block([[A, B.reshape((3, 1))], [0.0, 0.0, 0.0, 1.0]])


def aa_jacobian_sandbox():
  """ Axis-Angle Jacobian """
  s = np.array([1, 2, 3, 4, 5, 6])
  print(se3_to_screw(screw_to_se3(s)))

  aa = np.array([0.1, 0.0, 0.0])
  w, theta = aa_decomp(aa)
  print(f"w: {w}, theta: {theta}")

  h = 1e-8
  J_fdiff = np.zeros((3, 3))
  for i in range(3):
    # Forward difference
    w[i] += 0.5 * h
    aa_fwd = aa_vec(w, theta)
    w[i] -= 0.5 * h

    # Backward difference
    w[i] -= 0.5 * h
    aa_bwd = aa_vec(w, theta)
    w[i] += 0.5 * h

    # Central finite difference
    J_fdiff[:, i] = (aa_fwd - aa_bwd) / h

  print(f"J_fdiff: {J_fdiff}")

  h = 1e-8
  J_fdiff = np.zeros((3, 1))

  # Forward difference
  theta += 0.5 * h
  aa_fwd = aa_vec(w, theta)
  theta -= 0.5 * h

  # Backward difference
  theta -= 0.5 * h
  aa_bwd = aa_vec(w, theta)
  theta += 0.5 * h

  # Central finite difference
  J_fdiff[:, 0] = (aa_fwd - aa_bwd) / h

  print(f"J_fdiff: {J_fdiff}")


class RobotArmSandbox:
  """ Robot Arm Sandbox"""
  def __init__(self):
    self.calib_target, self.T_WF = self._setup_calib_target()
    self.cam_params = self._setup_camera_params()

    # Home configuration M
    offset_y = -1.0 * self.calib_target.get_dimensions()[0] / 2.0
    offset_z = self.calib_target.get_dimensions()[1] / 2.0
    L1 = 1.0
    L2 = offset_z
    self.M = np.array([
        [0.0, 0.0, 1.0, L1],
        [0.0, 1.0, 0.0, offset_y],
        [-1.0, 0.0, 0.0, L2],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Joint angles
    self.th1 = 0.0
    self.th2 = 0.0
    self.th3 = 0.0

    # Screw axis
    self.s1 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    self.s2 = np.array([0.0, -1.0, 0.0, 0.0, 0.0, -L1])
    self.s3 = np.array([1.0, 0.0, 0.0, 0.0, L2, 0.0])

    # End-effector to camera extrinsics
    C_EC = euler321(deg2rad(-90.0), 0.0, 0.0)
    r_EC = np.array([-0.1, 0.0, 0.0])
    self.T_EC = tf(C_EC, r_EC)

  @staticmethod
  def _setup_calib_target():
    """ Setup Calibration Target """
    calib_target = AprilGrid()
    C_WF = euler321(-pi / 2.0, 0.0, deg2rad(90.0))
    r_WF = np.array([2.0, 0.0, 0.0])
    T_WF = tf(C_WF, r_WF)
    return calib_target, T_WF

  @staticmethod
  def _setup_camera_params():
    """ Setup Camera Parameters """
    cam_idx = 0
    res = [640, 480]
    fov = 120.0
    fx = focal_length(res[0], fov)
    fy = focal_length(res[0], fov)
    cx = res[0] / 2.0
    cy = res[1] / 2.0

    proj_model = "pinhole"
    dist_model = "radtan4"
    proj_params = [fx, fy, cx, cy]
    dist_params = [0.0, 0.0, 0.0, 0.0]
    params = np.block([*proj_params, *dist_params])

    return camera_params_setup(cam_idx, res, proj_model, dist_model, params)

  def get_end_effector_tf(self):
    """ Get End-Effector Transform """
    T_B1 = poe(self.s1, self.th1)
    T_12 = poe(self.s2, self.th2)
    T_23 = poe(self.s3, self.th3)
    T_BE = T_B1 @ T_12 @ T_23 @ self.M
    return T_B1, T_BE

  def get_camera_measurements(self):
    """ Simulate camera frame """
    cam_geom = self.cam_params.data
    _, T_BE = self.get_end_effector_tf()
    T_WC = T_BE @ self.T_EC
    T_CW = np.linalg.inv(T_WC)

    keypoints = []
    object_points = self.calib_target.get_object_points()
    for r_FFi in object_points:
      r_C = tf_point(T_CW @ self.T_WF, r_FFi)
      status, z = cam_geom.project(self.cam_params.param, r_C)
      if status:
        keypoints.append(z)

    return np.array(keypoints)

  def plot_camera_frame(self):
    """ Plot camera frame """
    cam_geom = self.cam_params.data
    cam_res = cam_geom.resolution
    measurements = self.get_camera_measurements()

    plt.figure()
    ax = plt.subplot(111)
    ax.plot(measurements[:, 0], measurements[:, 1], 'r.')
    ax.set_xlim([0, cam_res[0]])
    ax.set_ylim([0, cam_res[1]])
    ax.set_xlabel('pixel')
    ax.set_ylabel('pixel')
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')
    plt.show()

  def visualize_scene(self):
    """ Visualize Scene """
    T_B1, T_BE = self.get_end_effector_tf()
    T_BC = T_BE @ self.T_EC

    # Visualize
    plt.figure()
    ax = plt.axes(projection='3d')
    self.calib_target.plot(ax, self.T_WF)
    plot_tf(ax, T_B1, name="Base", size=0.1)
    plot_tf(ax, T_BE, name="End", size=0.1)
    plot_tf(ax, T_BC, name="Camera", size=0.1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plot_set_axes_equal(ax)
    plt.show()


def main():
  """ Main function """
  sandbox = RobotArmSandbox()
  # sandbox.visualize_scene()
  # sandbox.plot_camera_frame()
  # measurements = sandbox.get_camera_measurements()

  cam_params = sandbox.cam_params.param
  cam_geom = sandbox.cam_params.data
  T_WF = sandbox.T_WF
  _, T_BE = sandbox.get_end_effector_tf()
  T_WC = T_BE @ sandbox.T_EC
  T_CW = np.linalg.inv(T_WC)
  p_FFi = sandbox.calib_target.get_object_points()[0]

  p_C = tf_point(T_CW @ T_WF, p_FFi)
  status, z = cam_geom.project(cam_params, p_C)
  print(f"status: {status}, z: {z}")

  # dr__dr_CFi = dr__dh @ dh__dx_dash @ dx_dash__dx @ dx__dp_CFi @ dp_CFi__dT_CB
  # p_CFi = inv(T_EC) @ inv(T_BE) @  inv(T_WB) @ T_WF

  # dp_CFi/dT_CE


if __name__ == "__main__":
  main()
