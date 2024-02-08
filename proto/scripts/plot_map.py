#!/usr/bin/env python3
import os
import sys

import proto
import pandas
import numpy as np
import matplotlib.pylab as plt


def load_poses(csv_path):
  """ Load poses """
  csv_data = pandas.read_csv(csv_path)
  pose_data = []

  for row_idx in range(csv_data.shape[0]):
    pose_ts = csv_data["ts"][row_idx]

    rx = csv_data["rx"][row_idx]
    ry = csv_data["ry"][row_idx]
    rz = csv_data["rz"][row_idx]
    r = np.array([rx, ry, rz])

    qw = csv_data["qw"][row_idx]
    qx = csv_data["qx"][row_idx]
    qy = csv_data["qy"][row_idx]
    qz = csv_data["qz"][row_idx]
    q = np.array([qw, qx, qy, qz])

    pose_data.append((pose_ts, proto.tf(q, r)))

  return pose_data


def load_traj_data(filename):
  """
  Reads a trajectory from a text file.

  File format:
  The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp
  (to be matched) and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D
  orientation) associated to this timestamp.

  Input:
  filename -- File name

  Output:
  dict -- dictionary of (stamp,data) tuples

  """
  file = open(filename)
  data = file.read()
  lines = data.replace(",", " ").replace("\t", " ").split("\n")

  traj_data = {}
  for line in lines:
    data = line.split(" ")
    if len(data) == 0 or data[0] == "":
      continue

    ts = int(data[0])
    rx = float(data[1])
    ry = float(data[2])
    rz = float(data[3])

    traj_data[ts] = [rx, ry, rz]

  return traj_data


def associate(first_list, second_list, t_offset, max_difference):
  """
    Associate two dictionaries of (stamp,data). As the time stamps never match
    exactly, we aim to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    t_offset -- time offset between both dictionaries (e.g., to model the delay
                between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

  """
  # first_keys = first_list.keys()
  # second_keys = second_list.keys()
  first_keys = list(first_list)
  second_keys = list(second_list)
  potential_matches = [(abs(a - (b + t_offset)), a, b)
                       for a in first_keys
                       for b in second_keys
                       if abs(a - (b + t_offset)) < max_difference]
  potential_matches.sort()
  matches = []
  for _, a, b in potential_matches:
    if a in first_keys and b in second_keys:
      first_keys.remove(a)
      second_keys.remove(b)
      matches.append((a, b))

  matches.sort()
  return matches


def align(model, data):
  """Align two trajectories using the method of Horn (closed-form).

    Input:

        model -- first trajectory (3xn)
        data -- second trajectory (3xn)

    Output:

        rot -- rotation matrix (3x3)
        trans -- translation vector (3x1)
        trans_error -- translational error per point (1xn)

    """
  np.set_printoptions(precision=3, suppress=True)
  model_zerocentered = model - model.mean(1)
  data_zerocentered = data - data.mean(1)

  W = np.zeros((3, 3))
  for column in range(model.shape[1]):
    W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
  U, _, Vh = np.linalg.linalg.svd(W.transpose())

  S = np.matrix(np.identity(3))
  if np.linalg.det(U) * np.linalg.det(Vh) < 0:
    S[2, 2] = -1
  rot = U * S * Vh

  rotmodel = rot * model_zerocentered
  dots = 0.0
  norms = 0.0

  for column in range(data_zerocentered.shape[1]):
    dots += np.dot(data_zerocentered[:, column].transpose(), rotmodel[:,
                                                                      column])
    normi = np.linalg.norm(model_zerocentered[:, column])
    norms += normi * normi

  s = float(dots / norms)

  transGT = data.mean(1) - s * rot * model.mean(1)
  trans = data.mean(1) - rot * model.mean(1)

  model_alignedGT = s * rot * model + transGT
  model_aligned = rot * model + trans

  alignment_errorGT = model_alignedGT - data
  alignment_error = model_aligned - data

  trans_errorGT = np.sqrt(
      np.sum(np.multiply(alignment_errorGT, alignment_errorGT), 0)).A[0]
  trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error),
                               0)).A[0]

  return rot, transGT, trans_errorGT, trans, trans_error, s, model_aligned


def eval_traj(gnd_file, est_file, **kwargs):
  verbose = kwargs.get("verbose", False)
  offset = 0.0
  max_diff = 0.02

  # Read files and associate them
  gnd_data = load_traj_data(gnd_file)
  est_data = load_traj_data(est_file)
  matches = associate(gnd_data, est_data, offset, max_diff)
  if len(matches) < 2:
    sys.exit("Matches < 2! Did you choose the correct sequence?")

  # Extract matches between ground-truth and estimates and form matrices
  gnd_mat = []
  est_mat = []
  for ts_a, ts_b in matches:
    gnd_data_row = [float(value) for value in gnd_data[ts_a][0:3]]
    est_data_row = [float(value) for value in est_data[ts_b][0:3]]
    gnd_mat.append(gnd_data_row)
    est_mat.append(est_data_row)
  gnd_mat = np.matrix(gnd_mat).transpose()
  est_mat = np.matrix(est_mat).transpose()

  # Align both estimates and ground-truth
  rot, transGT, trans_errorGT, trans, trans_error, scale = align(
      est_mat, gnd_mat)

  # Calculate errors
  metrics = {
      "ate": {
          "rmse": np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)),
          "mean": np.mean(trans_error),
          "median": np.median(trans_error),
          "std": np.std(trans_error),
          "min": np.min(trans_error),
          "max": np.max(trans_error)
      }
  }

  if verbose:
    print("compared_pose_pairs %d pairs" % (len(trans_error)))
    print("ATE.rmse %f m" % metrics["ate"]["rmse"])
    print("ATE.mean %f m" % metrics["ate"]["mean"])
    print("ATE.median %f m" % metrics["ate"]["median"])
    print("ATE.std %f m" % metrics["ate"]["std"])
    print("ATE.min %f m" % metrics["ate"]["min"])
    print("ATE.max %f m" % metrics["ate"]["max"])

  return metrics


if __name__ == "__main__":
  features_path = "/tmp/features.csv"
  poses_path = "/tmp/poses.csv"
  euroc_path = "/data/euroc/V1_01/mav0/state_groundtruth_estimate0/data.csv"

  est_path = "/tmp/est_data.csv"
  gnd_path = "/tmp/gnd_data.csv"


  # features = pandas.read_csv(features_path)
  # plt.figure()
  # ax = plt.axes(projection='3d')
  # ax.scatter3D(features["x"], features["y"], features["z"], alpha=0.1)
  # ax.set_xlim([-5, 5])
  # ax.set_ylim([-5, 5])
  # ax.set_zlim([0, 5])
  # # proto.plot_set_axes_equal(ax)
  # plt.show()

  # Read files and associate them
  offset = 0.0
  max_diff = 0.02
  gnd_data = load_traj_data(gnd_path)
  est_data = load_traj_data(est_path)
  matches = associate(gnd_data, est_data, offset, max_diff)
  if len(matches) < 2:
    sys.exit("Matches < 2! Did you choose the correct sequence?")

  # Extract matches between ground-truth and estimates and form matrices
  gnd_mat = []
  est_mat = []
  for ts_a, ts_b in matches:
    gnd_data_row = [float(value) for value in gnd_data[ts_a][0:3]]
    est_data_row = [float(value) for value in est_data[ts_b][0:3]]
    gnd_mat.append(gnd_data_row)
    est_mat.append(est_data_row)
  gnd_mat = np.matrix(gnd_mat).transpose()
  est_mat = np.matrix(est_mat).transpose()

  # Align both estimates and ground-truth
  rot, transGT, trans_errorGT, trans, trans_error, scale, model_aligned = align(
      est_mat, gnd_mat)

  est_data = {
      "rx": [],
      "ry": [],
      "rz": [],
  }
  for i in range(model_aligned.shape[1]):
    est_data["rx"].append(model_aligned[0, i])
    est_data["ry"].append(model_aligned[1, i])
    est_data["rz"].append(model_aligned[2, i])

  # for ts, T_WC0 in load_poses(poses_path):
  #   T_WS = T_WC0
  #   r_WS = proto.tf_trans(T_WS).reshape((3, 1))
  #   r_WS = rot @ r_WS - trans

  #   est_data["rx"].append(r_WS[0,0])
  #   est_data["ry"].append(r_WS[1,0])
  #   est_data["rz"].append(r_WS[2,0])

  # gnd_data = {
  #     "rx": [],
  #     "ry": [],
  #     "rz": [],
  # }
  # for ts, data in load_traj_data(gnd_path).items():
  #   r_WS = np.array(data[0:3])
  #   q_WS = np.array(data[3:8])
  #   T_WS = proto.tf(q_WS, r_WS)
  #   r_WS = proto.tf_trans(T_WS)
  #   gnd_data["rx"].append(r_WS[0])
  #   gnd_data["ry"].append(r_WS[1])
  #   gnd_data["rz"].append(r_WS[2])

  gnd_data = pandas.read_csv(euroc_path)
  plt.plot(est_data["rx"], est_data["ry"], "r-", label="Estimate")
  plt.plot(gnd_data[" p_RS_R_x [m]"], gnd_data[" p_RS_R_y [m]"], "k--", label="Ground-Truth")
  plt.show()
