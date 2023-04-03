#!/usr/bin/env python3
import os
from pathlib import Path
import glob
import yaml
from dataclasses import dataclass

import numpy as np
from proto import *


def fix_pose(pose):
    """ Fix pose vector ordering """
    rx, ry, rz, qw, qx, qy, qz = pose
    return np.array([rx, ry, rz, qx, qy, qz, qw])

@dataclass
class GimbalData:
    """ Gimbal Data"""
    num_cams: int
    num_views: int
    num_poses: int
    num_links: int
    num_joints: int
    timestamps: np.ndarray
    fiducial_ext: np.ndarray
    gimbal_ext: np.ndarray
    cam0_params: np.ndarray
    cam1_params: np.ndarray
    cam0_ext: np.ndarray
    cam1_ext: np.ndarray
    link0_ext: np.ndarray
    link1_ext: np.ndarray
    joints: dict
    poses: dict

    def get_plot_tfs(self, ts):
        """ Get plot transforms """
        # links
        links = []
        links.append(self.link0_ext)
        links.append(self.link1_ext)

        # Joints
        joint_angles = []
        joint_angles.append(self.joints[ts][0])
        joint_angles.append(self.joints[ts][1])
        joint_angles.append(self.joints[ts][2])

        # Gimbal
        gimbal = GimbalKinematics(links, joint_angles)
        T_M0L0 = gimbal.forward_kinematics(joint_idx=0)
        T_M0L1 = gimbal.forward_kinematics(joint_idx=1)
        T_M0L2 = gimbal.forward_kinematics(joint_idx=2)

        # Transforms
        first_ts = self.timestamps[0]
        T_WB = pose2tf(self.poses[first_ts])
        T_BM0 = pose2tf(self.gimbal_ext)
        T_WL0 = T_WB @ T_BM0 @ T_M0L0
        T_WL1 = T_WB @ T_BM0 @ T_M0L1
        T_WL2 = T_WB @ T_BM0 @ T_M0L2
        T_WC0 = T_WL2 @ pose2tf(self.cam0_ext)
        T_WC1 = T_WL2 @ pose2tf(self.cam1_ext)

        return (T_WL0, T_WL1, T_WL2, T_WC0, T_WC1)


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

        proj_params = config_data[f"cam{cam_idx}"]["proj_params"]
        dist_params = config_data[f"cam{cam_idx}"]["dist_params"]
        cam_params = np.array([*proj_params, *dist_params])
        config_data[f"cam{cam_idx}_params"] = cam_params

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
    ts = int(Path(view_file).stem)
    view_file = open(view_file, "r")
    view_lines = view_file.readlines()
    num_corners = parse_int(view_lines[0], "num_corners")

    view_data = {}
    view_data["ts"] = ts
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
    pose_data = {}
    pose_data["num_poses"] = num_poses
    for line in lines[3:]:
        line = line.split(",")
        ts = int(line[0])
        rx, ry, rz, qw, qx, qy, qz = np.array(line[1:], dtype=float)
        pose_data[ts] = np.array([rx, ry, rz, qx, qy, qz, qw])

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

    timestamps = []
    for line in lines[4:]:
        line = line.split(",")
        ts = int(line[0])
        joint0 = float(line[1])
        joint1 = float(line[2])
        joint2 = float(line[3])

        timestamps.append(ts)
        joints_data[ts] = np.array([joint0, joint1, joint2])

    return timestamps, joints_data


def load_sim_gimbal_data(calib_dir):
    """ Load gimbal simulation data. """
    config_path = os.path.join(calib_dir, "calib.config")
    poses_path = os.path.join(calib_dir, "poses.sim")
    joints_path = os.path.join(calib_dir, "joint_angles.sim")

    config_data = load_config_file(config_path)
    poses_data = load_poses_file(poses_path)
    timestamps, joints_data = load_joints_file(joints_path)

    cam_dir = os.path.join(calib_dir, "cam0")
    cam_files = glob.glob(os.path.join(cam_dir, "*.sim"))
    cam_files = sorted(cam_files, key= lambda x : int(Path(x).stem))
    view_data = load_view_file(cam_files[0])

    calib_data = {}
    calib_data["config"] = config_data
    calib_data["poses"] = poses_data
    calib_data["joints"] = joints_data
    calib_data["views"] = view_data


    return GimbalData(
        config_data["num_cams"],
        joints_data["num_views"],
        poses_data["num_poses"],
        config_data["num_links"],
        joints_data["num_joints"],
        timestamps,
        config_data["fiducial_ext"],
        config_data["gimbal_ext"],
        config_data["cam0_params"],
        config_data["cam1_params"],
        config_data["cam0_ext"],
        config_data["cam1_ext"],
        config_data["link0_ext"],
        config_data["link1_ext"],
        joints_data,
        poses_data
    ), calib_data

def load_calib_results(results_file):
    """ Load calib results """
    num_cams = None
    num_views = None
    num_poses = None
    num_links = None
    num_joints = None

    fiducial_ext = None
    gimbal_ext = None
    cam0_params = None
    cam1_params = None
    link0_ext = None
    link1_ext = None
    joints = {}
    poses = {}

    with open(results_file, 'r') as file:
        data = yaml.safe_load(file)

        num_cams = data["num_cams"]
        num_views = data["num_views"]
        num_poses = data["num_poses"]
        num_links = data["num_links"]
        num_joints = data["num_joints"]

        fiducial_ext = fix_pose(data["fiducial_ext"])
        gimbal_ext = fix_pose(data["gimbal_ext"])
        cam0_params = data["cam0_params"]
        cam1_params = data["cam1_params"]
        cam0_ext = fix_pose(data["cam0_ext"])
        cam1_ext = fix_pose(data["cam1_ext"])
        link0_ext = fix_pose(data["link0_ext"])
        link1_ext = fix_pose(data["link1_ext"])

        stride = num_joints + 1
        for k in range(num_views):
            idx = k * stride
            ts = data["joints"][idx]
            joints[ts] = [j for j in data["joints"][idx + 1:idx + 4]]

        stride = 7 + 1
        for k in range(num_poses):
            idx = k * stride
            ts = data["poses"][idx]
            poses[ts] = fix_pose([j for j in data["poses"][idx + 1:idx + 8]])


    return GimbalData(
        num_cams,
        num_views,
        num_poses,
        num_links,
        num_joints,
        fiducial_ext,
        gimbal_ext,
        cam0_params,
        cam1_params,
        cam0_ext,
        cam1_ext,
        link0_ext,
        link1_ext,
        joints,
        poses
    )


def plot_gimbal(ts, **kwargs):
    """ Plot Gimbal """
    gnd = kwargs.get("gnd")
    init = kwargs.get("init", None)
    est = kwargs.get("est", None)

    # Visualize
    plt.figure()
    ax = plt.axes(projection='3d')

    # Plot transforms
    (T_WL0, T_WL1, T_WL2, T_WC0, T_WC1) = gnd.get_plot_tfs(ts)
    gnd_colors = ['r-', 'g-', 'b-'] if est is None else ['r-', 'r-', 'r-']
    plot_tf(ax, T_WL0, name="Link0", size=0.05, colors=gnd_colors)
    plot_tf(ax, T_WL1, name="Link1", size=0.05, colors=gnd_colors)
    plot_tf(ax, T_WL2, name="Link2", size=0.05, colors=gnd_colors)
    plot_tf(ax, T_WC0, name="cam0", size=0.05, colors=gnd_colors)
    plot_tf(ax, T_WC1, name="cam1", size=0.05, colors=gnd_colors)

    if est:
        (T_WL0, T_WL1, T_WL2, T_WC0, T_WC1) = est.get_plot_tfs(ts)
        est_colors = ['b-', 'b-', 'b-']
        plot_tf(ax, T_WL0, name="Link0", size=0.05, colors=est_colors)
        plot_tf(ax, T_WL1, name="Link1", size=0.05, colors=est_colors)
        plot_tf(ax, T_WL2, name="Link2", size=0.05, colors=est_colors)
        plot_tf(ax, T_WC0, name="cam0", size=0.05, colors=est_colors)
        plot_tf(ax, T_WC1, name="cam1", size=0.05, colors=est_colors)

    if init:
        (T_WL0, T_WL1, T_WL2, T_WC0, T_WC1) = init.get_plot_tfs(ts)
        est_colors = ['g-', 'g-', 'g-']
        plot_tf(ax, T_WL0, name="Link0", size=0.05, colors=est_colors)
        plot_tf(ax, T_WL1, name="Link1", size=0.05, colors=est_colors)
        plot_tf(ax, T_WL2, name="Link2", size=0.05, colors=est_colors)
        plot_tf(ax, T_WC0, name="cam0", size=0.05, colors=est_colors)
        plot_tf(ax, T_WC1, name="cam1", size=0.05, colors=est_colors)

    # Plot settings
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plot_set_axes_equal(ax)
    plt.show()


def check_transforms(sim_data, calib_data):
    """ Check transforms """
    # Gimbal
    view_data = calib_data["views"]
    view_ts = view_data["ts"]

    links = [sim_data.link0_ext, sim_data.link1_ext]

    joint_angles = [
        sim_data.joints[view_ts][0],
        sim_data.joints[view_ts][1],
        sim_data.joints[view_ts][2]
    ]

    gimbal = GimbalKinematics(links, joint_angles)
    T_M0L2 = gimbal.forward_kinematics(joint_idx=2)
    T_WB = pose2tf(sim_data.poses[sim_data.timestamps[0]])
    T_BM0 = pose2tf(sim_data.gimbal_ext)
    T_WL2 = T_WB @ T_BM0 @ T_M0L2
    T_C0W = tf_inv(T_WL2 @ pose2tf(sim_data.cam0_ext))
    T_C1W = tf_inv(T_WL2 @ pose2tf(sim_data.cam1_ext))

    T_WF = pose2tf(sim_data.fiducial_ext)
    reproj_errors = []
    for i in range(view_data["num_corners"]):
        z = view_data["keypoints"][i]
        p_F = view_data["object_points"][i]
        p_C = tf_point(T_C0W @ T_WF, p_F)

        proj_params = sim_data.cam0_params[0:4]
        dist_params = sim_data.cam0_params[4:8]
        z_hat = pinhole_radtan4_project(proj_params, dist_params, p_C)
        r = z - z_hat

        reproj_errors.append(sqrt(r @ r))

    reproj_errors = np.array(reproj_errors).flatten()
    print(reproj_errors.mean())



if __name__ == "__main__":
    # calib_dir = "./test_data/sim_gimbal"
    calib_dir = "/tmp/calib_gimbal"
    sim_data, calib_data  = load_sim_gimbal_data(calib_dir)
    # plot_gimbal(sim_data.timestamps[0], gnd=sim_data)
    # check_transforms(sim_data, calib_data)

    # results_file = "/tmp/estimates-gnd.yaml"
    # data_gnd = load_calib_results(results_file)

    # results_file = "/tmp/estimates-before.yaml"
    # data_before = load_calib_results(results_file)

    # results_file = "/tmp/estimates-after.yaml"
    # data_after = load_calib_results(results_file)

    # for k in range(data_gnd.num_views):
    #     plot_gimbal(k, gnd=data_gnd, init=data_before, est=data_after)
