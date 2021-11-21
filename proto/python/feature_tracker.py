# /usr/bin/env python3
import os
import sys
import glob

import cv2
import numpy as np

data_path = '/data/euroc/raw/V1_01'


class FeatureTrackingData:
  def __init__(self, image, feature_ids, keypoints, descriptors):
    self.image = image
    self.feature_ids = feature_ids
    self.keypoints = keypoints
    self.descriptors = descriptors


class FeatureMatches:
  def __init__(self, data_i, data_j):
    self.image_i = data_i.image
    self.keypoints_i = data_i.keypoints
    self.descriptors_i = data_i.descriptors

    self.image_j = data_j.image
    self.keypoints_j = data_j.keypoints
    self.descriptors_j = data_j.descriptors


class FeatureTracker:
  def __init__(self):
    # Settings
    # self.mode = "OVERLAPS_ONLY"
    self.mode = "OVERLAPS_ALLOWED"
    # self.mode = "NO_OVERLAPS_ALLOWED"

    # Feature detector, descriptor and matcher
    self.feature = cv2.ORB_create(nfeatures=50)
    self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Data
    self.frame_idx = 0
    self.features_detected = 0
    self.prev_camera_images = None
    self.cam_indices = []
    self.cam_overlaps = []
    self.cam_data = {}

  def add_camera(self, cam_idx):
    self.cam_indices.append(cam_idx)
    self.cam_data[cam_idx] = {"keypoints": None, "descriptors": None}

  def add_overlap(self, cam_i_idx, cam_j_idx):
    self.cam_overlaps.append((cam_i_idx, cam_j_idx))

  def _detect(self, image):
    kps = self.feature.detect(image, None)
    kps, des = self.feature.compute(image, kps)
    return (kps, des)

  def _match(self, data_i, data_j):
    # Match
    img_i, kps_i, des_i = data_i
    img_j, kps_j, des_j = data_j
    matches = self.matcher.match(des_i, des_j)
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

    data_i = (img_i, kps_i_new, des_i_new)
    data_j = (img_j, kps_j_new, des_j_new)
    return (data_i, data_j, matches_new)

  def _initialize(self, camera_images):
    # Detect features for each camera
    det_data = {}
    for cam_idx in self.cam_indices:
      image = camera_images[cam_idx]
      kps, des = self._detect(image)
      det_data[cam_idx] = {'image': image, 'keypoints': kps, 'descriptors': des}

    # Track overlaps only
    if self.mode == "OVERLAPS_ONLY":
      for cam_i, cam_j in self.cam_overlaps:
        # Image i, keypoints i and descriptors i
        img_i = det_data[cam_i]['image']
        kps_i = det_data[cam_i]['keypoints']
        des_i = det_data[cam_i]['descriptors']
        data_i = (img_i, kps_i, des_i)

        # Image j, keypoints j and descriptors j
        img_j = det_data[cam_j]['image']
        kps_j = det_data[cam_j]['keypoints']
        des_j = det_data[cam_j]['descriptors']
        data_j = (img_j, kps_j, des_j)

        # Match keypoints and descriptors
        pair_data = self._match(data_i, data_j)
        ((img_i, kps_i, des_i), (img_j, kps_j, des_j), matches) = pair_data

        # Add to camera data
        nb_features_k = len(kps_i) + len(kps_j)
        self.features_detected += nb_features_k
        start_idx = self.features_detected - nb_features_k
        end_idx = start_idx + nb_features_k

        self.cam_data[cam_i]['image'] = img_i
        self.cam_data[cam_i]['feature_ids'] = range(start_idx, end_idx)
        self.cam_data[cam_i]['keypoints'] = kps_i
        self.cam_data[cam_i]['descriptors'] = des_i

        self.cam_data[cam_j]['image'] = img_j
        self.cam_data[cam_j]['feature_ids'] = range(start_idx, end_idx)
        self.cam_data[cam_j]['keypoints'] = kps_j
        self.cam_data[cam_j]['descriptors'] = des_j

        viz = cv2.drawMatches(img_i, kps_i, img_j, kps_j, matches, None)
      cv2.imshow('viz', viz)
      cv2.waitKey(0)

    # Track additional overlaps
    elif self.mode == "OVERLAPS_ALLOWED":
      # TODO: FIX THE BELOW SO OVERLAPS ARE ALLOWED
      # Add to camera data
      for cam_idx in det_data.keys():
        image = det_data[cam_idx]['image']
        kps = det_data[cam_idx]['keypoints']
        des = det_data[cam_idx]['descriptors']

        nb_features_k = len(kps)
        self.features_detected += nb_features_k
        start_idx = self.features_detected - nb_features_k
        end_idx = start_idx + nb_features_k

        self.cam_data[cam_idx]['image'] = image
        self.cam_data[cam_idx]['feature_ids'] = range(start_idx, end_idx)
        self.cam_data[cam_idx]['keypoints'] = kps
        self.cam_data[cam_idx]['descriptors'] = des

    # Track each cameras individually
    elif self.mode == "NO_OVERLAPS_ALLOWED":
      # Add to camera data
      for cam_idx in det_data.keys():
        image = det_data[cam_idx]['image']
        kps = det_data[cam_idx]['keypoints']
        des = det_data[cam_idx]['descriptors']

        nb_features_k = len(kps)
        self.features_detected += nb_features_k
        start_idx = self.features_detected - nb_features_k
        end_idx = start_idx + nb_features_k

        self.cam_data[cam_idx]['image'] = image
        self.cam_data[cam_idx]['feature_ids'] = range(start_idx, end_idx)
        self.cam_data[cam_idx]['keypoints'] = kps
        self.cam_data[cam_idx]['descriptors'] = des

    else:
      raise RuntimeError("Invalid FeatureTracker mode [%s]!" % self.mode)

    # Keep camera images
    self.prev_camera_images = camera_images

  def _track_through_time(self, ts, camera_images):
    # Track features through time
    viz = []
    for cam_idx in self.cam_indices:
      # Previous image, keypoints and descriptors
      img_km1 = self.prev_camera_images[cam_idx]
      kps_km1 = self.cam_data[cam_idx]['keypoints']
      des_km1 = self.cam_data[cam_idx]['descriptors']
      data_km1 = (img_km1, kps_km1, des_km1)

      # Current image, keypoints and descriptors
      img_k = camera_images[cam_idx]
      kps_k, des_k = self._detect(img_k)
      data_k = (img_k, kps_k, des_k)

      # Match keypoints from past to current
      pair_data = self._match(data_km1, data_k)
      ((img_km1, kps_km1, des_km1), (img_k, kps_k, des_k), matches) = pair_data
      viz.append(cv2.drawMatches(img_km1, kps_km1, img_k, kps_k, matches, None))

    # Form visualization image
    return cv2.vconcat(viz)

  def update(self, ts, camera_images):
    # Track features
    viz = None
    if self.frame_idx == 0:
      self._initialize(camera_images)
    else:
      viz = self._track_through_time(ts, camera_images)

    # Update
    self.frame_idx += 1

    return viz


def load_euroc_dataset(data_path):
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


if __name__ == '__main__':
  # Load EuRoC dataset
  dataset = load_euroc_dataset(data_path)

  # Loop through dataset
  feature_tracker = FeatureTracker()
  feature_tracker.add_camera('cam0')
  feature_tracker.add_camera('cam1')
  feature_tracker.add_overlap('cam0', 'cam1')

  for ts in dataset['timestamps']:
    # Load images
    img0 = cv2.imread(dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    # Feed camera images to feature tracker
    camera_images = {}
    camera_images['cam0'] = img0
    camera_images['cam1'] = img1
    viz = feature_tracker.update(ts, camera_images)

    # Visualize
    sys.stdout.flush()
    if viz is not None:
      cv2.imshow('viz', viz)
      if cv2.waitKey(0) == ord('q'):
        break
