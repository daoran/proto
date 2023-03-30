#!/usr/bin/env python3
import os
from pathlib import Path
import glob
import yaml
from yaml.loader import SafeLoader

import numpy as np
from proto import *


def parse_int(line, key):
    """ Parse Integer """
    k, v = line.split(":")
    if k != key:
        raise RuntimeError(f"Key [{k} != {key}]")
    return int(v)


def parse_pose(line, key):
    """ Parse Vector """
    k, v = line.split(":")
    if k != key:
        raise RuntimeError(f"Key [{k} != {key}]")

    line = line.strip().split(":")[1]
    line = line.replace("[", "").replace("]", "").split(",")
    rx, ry, rz, qw, qx, qy, qz = np.array(line, dtype=float)
    return np.array([rx, ry, rz, qx, qy, qz, qw])


def parse_camera(config_file, cam_idx):
    """ Parse camera config """
    config = open(config_file, "r")
    lines = config.readlines()

    found = False
    resolution = None
    proj_model = None
    dist_model = None
    proj_params = None
    dist_params = None
    for line in lines:
        if line.strip() == f"cam{cam_idx}:":
            found = True
            continue

        if found and line.strip().split(":")[0] == "resolution":
            line = line.strip().split(":")[1]
            line = line.replace("[", "").replace("]", "").split(",")
            resolution = np.array(line, dtype=int)
        elif found and line.strip().split(":")[0] == "proj_model":
            proj_model = line.strip().split(":")[1].strip().replace("\"", "")
        elif found and line.strip().split(":")[0] == "dist_model":
            dist_model = line.strip().split(":")[1].strip().replace("\"", "")
        elif found and line.strip().split(":")[0] == "proj_params":
            line = line.strip().split(":")[1]
            line = line.replace("[", "").replace("]", "").split(",")
            proj_params = np.array(line, dtype=float)
        elif found and line.strip().split(":")[0] == "dist_params":
            line = line.strip().split(":")[1]
            line = line.replace("[", "").replace("]", "").split(",")
            dist_params = np.array(line, dtype=float)

    cam_data = {}
    cam_data["cam_idx"] = cam_idx
    cam_data["resolution"] = resolution
    cam_data["proj_model"] = proj_model
    cam_data["dist_model"] = dist_model
    cam_data["proj_params"] = proj_params
    cam_data["dist_params"] = dist_params

    return cam_data


def load_config_file(config_file):
    """ Load config file """
    config = open(config_file, "r")
    lines = config.readlines()

    config_data = {}
    config_data["num_cams"] = parse_int(lines[0], "num_cams")
    config_data["num_links"] = parse_int(lines[1], "num_links")
    for cam_idx in range(config_data["num_cams"]):
        config_data[f"cam{cam_idx}"] = parse_camera(config_file, cam_idx)

    target_keys = [f"link{i}_ext" for i in range(config_data["num_links"])]
    target_keys += [f"cam{i}_ext" for i in range(config_data["num_cams"])]
    target_keys += ["fiducial_ext", "gimbal_ext"]

    for line in lines:
        key = line.split(":")[0]
        if key in target_keys:
            target_key = target_keys[target_keys.index(key)]
            config_data[target_key] = parse_pose(line, target_key)

    return config_data



def load_view_file(view_file):
    """ Load view file """
    view_file = open(view_file, "r")
    view_lines = view_file.readlines()
    num_corners = parse_int(view_lines[0], "num_corners")

    view_data = {}
    view_data["num_corners"] = num_corners
    view_data["tag_ids"] = []
    view_data["corner_indices"] = []
    view_data["object_points"] = []
    view_data["keypoints"] = []

    assert(len(view_lines[3:]) == num_corners)
    for line in view_lines[3:]:
        line = line.split(",")
        view_data["tag_ids"].append(int(line[0]))
        view_data["corner_indices"].append(int(line[1]))
        view_data["object_points"].append(np.array(line[2:5], dtype=float))
        view_data["keypoints"].append(np.array(line[5:7], dtype=float))

    return view_data


def load_poses_file(poses_file):
    """ Parse pose file """
    poses_file = open(poses_file, "r")
    lines = poses_file.readlines()
    num_poses = parse_int(lines[0], "num_poses")

    assert(len(lines[3:]) == num_poses)
    pose_data = []
    for line in lines[3:]:
        rx, ry, rz, qw, qx, qy, qz = np.array(line.split(","), dtype=float)
        pose_data.append(np.array([rx, ry, rz, qx, qy, qz, qw]))

    return pose_data


def load_joints_file(joints_file):
    """ Load joints file """
    joints_file = open(joints_file, "r")
    lines = joints_file.readlines()
    num_views = parse_int(lines[0], "num_views")
    num_joints = parse_int(lines[1], "num_joints")

    assert(len(lines[4:]) == num_views)
    joints_data = {}
    joints_data["num_views"] = num_views
    joints_data["num_joints"] = num_joints

    for line in lines[4:]:
        line = line.split(",")
        ts = int(line[0])
        joint0 = float(line[1])
        joint1 = float(line[2])
        joint2 = float(line[3])

        joints_data[ts] = np.array([joint0, joint1, joint2])

    return joints_data


def plot_gimbal(config_data, poses_data, joints_data):
    """ Plot Gimbal """
    # links
    links = []
    links.append(config_data["link0_ext"])
    links.append(config_data["link1_ext"])

    # Joints
    joint_angles = []
    joint_angles.append(0)
    joint_angles.append(0)
    joint_angles.append(0)

    # Gimbal
    gimbal = GimbalKinematics(links, joint_angles)
    T_M0L0 = gimbal.forward_kinematics(joint_idx=0)
    T_M0L1 = gimbal.forward_kinematics(joint_idx=1)
    T_M0L2 = gimbal.forward_kinematics(joint_idx=2)

    # Transforms
    T_WB = pose2tf(poses_data[0])
    T_BM0 = pose2tf(config_data["gimbal_ext"])
    T_WL0 = T_WB @ T_BM0 @ T_M0L0
    T_WL1 = T_WB @ T_BM0 @ T_M0L1
    T_WL2 = T_WB @ T_BM0 @ T_M0L2
    T_WC0 = T_WL2 @ pose2tf(config_data["cam0_ext"])
    T_WC1 = T_WL2 @ pose2tf(config_data["cam1_ext"])

    # Visualize
    plt.figure()
    ax = plt.axes(projection='3d')

    # Plot transforms
    plot_tf(ax, T_WL0, name="Link0", size=0.05)
    plot_tf(ax, T_WL1, name="Link1", size=0.05)
    plot_tf(ax, T_WL2, name="Link2", size=0.05)
    plot_tf(ax, T_WC0, name="cam0", size=0.05)
    plot_tf(ax, T_WC1, name="cam1", size=0.05)

    # Plot settings
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plot_set_axes_equal(ax)
    plt.show()


if __name__ == "__main__":
    calib_data = "./test_data/sim_gimbal"
    config_path = os.path.join(calib_data, "calib.config")
    poses_path = os.path.join(calib_data, "poses.sim")
    joints_path = os.path.join(calib_data, "joint_angles.sim")

    config_data = load_config_file(config_path)
    poses_data = load_poses_file(poses_path)
    joints_data = load_joints_file(joints_path)

    cam_dir = os.path.join(calib_data, "cam0")
    cam_files = glob.glob(os.path.join(cam_dir, "*.sim"))
    cam_files = sorted(cam_files, key= lambda x : int(Path(x).stem))
    view_data = load_view_file(cam_files[0])

    plot_gimbal(config_data, poses_data, joints_data)
