# /usr/bin/env python3
import os
import sys
import glob
import math
import unittest

import cv2
import numpy as np

data_path = '/data/euroc/raw/V1_01'


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


def klt(img_i, img_j, kps_i):
  """
  Track keypoints from image i to image j using optical flow.
  """
  # Track using optical flow
  patch_size = 10
  criteria = cv2.TermCriteria_COUNT
  criteria |= cv2.TermCriteria_EPS
  criteria |= max_iter
  criteria |= epsilon

  config = {}
  config['windSize'] = (patch_size, patch_size)
  config['maxLevel'] = 2
  config['criteria'] = criteria
  config['flags'] = cv2.OPTFLOW_USE_INITIAL_FLOW

  track_results = cv2.calcOpticalFlowPyrLK(img_i, img_j, kps_i, None, criteria)
  (kps_j, optflow_mask, errs) = track_results

  # Remove outliers using RANSAC
  F, fundmat_mask = cv2.findFundamentalMat(kps_i, kps_j, cv2.FM_8POINT)

  # Update or mark feature as lost
  inliers = []
  for i in range(len(kps_i)):
    if optflow_mask[i] and fundmat_mask[i]:
      inliers.append(True)
    else:
      inliers.append(False)

  return (kps_i, kps_j, inliers)


def grid_detect(detector, image, **kwargs):
  """
  Detect features uniformly using a grid system.
  """
  max_keypoints = kwargs.get('max_keypoints', 1000)
  grid_rows = kwargs.get('grid_rows', 3)
  grid_cols = kwargs.get('grid_cols', 3)

  # Calculate number of grid cells and max corners per cell
  image_height, image_width = image.shape
  dx = int(math.ceil(float(image_width) / float(grid_cols)))
  dy = int(math.ceil(float(image_height) / float(grid_rows)))
  nb_cells = grid_rows * grid_cols
  max_per_cell = math.floor(max_keypoints / nb_cells)

  # Detect corners in each grid cell
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
      for i in range(min(len(kps), max_per_cell)):
        kp = kps[i]
        kp.pt = (kp.pt[0] + x, kp.pt[1] + y)
        kps_all.append(kp)
        des_all.append(des[i, :])

      cell_idx += 1

  viz = cv2.drawKeypoints(image, kps_all, None)
  cv2.imshow("viz", viz)
  cv2.waitKey(0)

  return kps_all, np.array(des_all)


class FeaturesGrid:
  def __init__(self, grid_rows, grid_cols, image_shape, keypoints, descriptors):
    self.keypoints = keypoints
    self.descriptors = descriptors
    self.bins = []

  def count(self, cell_idx):
    pass


class FeatureTrackingData:
  def __init__(self, image, keypoints, descriptors=None, feature_ids=None):
    self.image = image
    self.keypoints = keypoints
    self.descriptors = descriptors
    self.feature_ids = feature_ids

  def get_all(self):
    return (self.image, self.keypoints, self.descriptors, self.feature_ids)


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
    self.mode = "OVERLAPS_ONLY"
    # self.mode = "NON_OVERLAPS_ONLY"
    # self.mode = "DEFAULT"

    # Feature detector, descriptor and matcher
    self.feature = cv2.ORB_create(nfeatures=100)
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
    assert image is not None
    # grid_detect(self.feature, image)

    kps = self.feature.detect(image, None)
    kps, des = self.feature.compute(image, kps)
    ft_data = FeatureTrackingData(image, kps, des)

    return ft_data

  def _detect_multicam(self, camera_images):
    det_data = {}
    for cam_idx in self.cam_indices:
      image = camera_images[cam_idx]
      det_data[cam_idx] = self._detect(image)

    return det_data

  def _match(self, data_i, data_j, sort_matches=True):
    # Match
    (img_i, kps_i, des_i, _) = data_i.get_all()
    (img_j, kps_j, des_j, _) = data_j.get_all()
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

    # Form return data
    data_i = FeatureTrackingData(img_i, kps_i_new, des_i_new)
    data_j = FeatureTrackingData(img_j, kps_j_new, des_j_new)
    return (data_i, data_j, matches_new)

  def _form_feature_ids(self, nb_kps):
    nb_features_k = nb_kps
    self.features_detected += nb_features_k
    start_idx = self.features_detected - nb_features_k
    end_idx = start_idx + nb_features_k
    return range(start_idx, end_idx)

  def _initialize(self, camera_images):
    # Track both overlapping and non-overlapping feature
    if self.mode == "DEFAULT":
      # Detect features for each camera
      det_data = self._detect_multicam(camera_images)

      # Track overlaps
      for cam_i, cam_j in self.cam_overlaps:
        # Match keypoints and descriptors
        data_i = det_data[cam_i]
        data_j = det_data[cam_j]
        (data_i, data_j, matches) = self._match(data_i, data_j)

        # Add to camera data
        feature_ids = self._form_feature_ids(len(matches))
        data_i.feature_ids = feature_ids
        data_j.feature_ids = feature_ids
        self.cam_data[cam_i] = data_i
        self.cam_data[cam_j] = data_j

      # Track non-overlaps
      det_data = self._detect_multicam(camera_images)
      for cam_idx in det_data.keys():
        data = det_data[cam_idx]
        data.feature_ids = _form_feature_ids(len(kps))
        self.cam_data[cam_idx] = data

    # Track overlaps only
    if self.mode == "OVERLAPS_ONLY":
      # Detect features for each camera
      det_data = self._detect_multicam(camera_images)

      # Match overlapping features and add to camera data
      for cam_i, cam_j in self.cam_overlaps:
        # Match keypoints and descriptors
        data_i = det_data[cam_i]
        data_j = det_data[cam_j]
        (data_i, data_j, matches) = self._match(data_i, data_j)

        # Add to camera data
        feature_ids = self._form_feature_ids(len(matches))
        data_i.feature_ids = feature_ids
        data_j.feature_ids = feature_ids
        self.cam_data[cam_i] = data_i
        self.cam_data[cam_j] = data_j

    # Track each cameras individually
    elif self.mode == "NON_OVERLAPS_ONLY":
      # Detect features for each camera
      det_data = self._detect_multicam(camera_images)

      # Add to camera data
      for cam_idx in det_data.keys():
        data = det_data[cam_idx]
        data.feature_ids = _form_feature_ids(len(kps))
        self.cam_data[cam_idx] = data

    # Invalid feature tracker mode
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
      data_km1 = self.cam_data[cam_idx]

      # Current image, keypoints and descriptors
      img_k = camera_images[cam_idx]
      data_k = self._detect(img_k)

      # Match keypoints from past to current
      (data_km1, data_k, matches) = self._match(data_km1, data_k)
      (img_km1, kps_km1, _, _) = data_km1.get_all()
      (img_k, kps_k, _, _) = data_k.get_all()
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


class TestFeatureTracker(unittest.TestCase):
  def setUp(self):
    # Load test data
    self.dataset = load_euroc_dataset(data_path)

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
    # Load images
    ts = self.dataset['timestamps'][0]
    img0 = cv2.imread(self.dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(self.dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    # Feed camera images to feature tracker
    camera_images = {}
    camera_images['cam0'] = img0
    camera_images['cam1'] = img1
    self.feature_tracker._detect_multicam(camera_images)

  def test_run(self):
    for ts in self.dataset['timestamps']:
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
      if viz is not None:
        cv2.imshow('viz', viz)
        if cv2.waitKey(0) == ord('q'):
          break


if __name__ == '__main__':
  unittest.main()
