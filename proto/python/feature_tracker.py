# /usr/bin/python3
import os
import glob

import cv2

data_path = '/data/euroc/raw/V1_01'


class Feature(object):
  def __init__(self):
    self.data = {}

  def add_keypoint(self, ts, cam_idx, kp):
    if ts in self.data:
      self.data[ts] = {}
      self.data[ts][cam_idx] = kp
    else:
      self.data[ts][cam_idx] = kp

  def seen_at(self, ts):
    return True if ts in self.data else False


class FeatureTracker(object):
  def __init__(self):
    self.data = []

  def update(self, camera_images):
    return True


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
  dataset = load_euroc_dataset(data_path)

  for ts in dataset['timestamps']:
    img0 = cv2.imread(dataset['cam0'][ts], cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(dataset['cam1'][ts], cv2.IMREAD_GRAYSCALE)

    cv2.imshow('viz', img0)
    if cv2.waitKey(1) == ord('q'):
      break
