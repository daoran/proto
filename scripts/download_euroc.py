#!/usr/bin/env python3
import os
import glob
import urllib.request
import zipfile


def download(src, dst):
  """ Download """
  if not os.path.exists(dst):
    print(f"Downloading [{src}] -> [{dst}]", flush=True)
    urllib.request.urlretrieve(src, dst)


def download_sequences(dst_dir):
  """ Download sequences """
  base_url = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
  seqs = [
      "calibration_datasets/cam_april/cam_april.zip",
      "calibration_datasets/imu_april/imu_april.zip",
      "machine_hall/MH_01_easy/MH_01_easy.zip",
      "machine_hall/MH_02_easy/MH_02_easy.zip",
      "machine_hall/MH_03_medium/MH_03_medium.zip",
      "machine_hall/MH_04_difficult/MH_04_difficult.zip",
      "machine_hall/MH_05_difficult/MH_05_difficult.zip",
      "machine_hall/MH_01_easy/MH_01_easy.zip",
      "machine_hall/MH_02_easy/MH_02_easy.zip",
      "machine_hall/MH_03_medium/MH_03_medium.zip",
      "machine_hall/MH_04_difficult/MH_04_difficult.zip",
      "machine_hall/MH_05_difficult/MH_05_difficult.zip",
      "vicon_room1/V1_01_easy/V1_01_easy.zip",
      "vicon_room1/V1_02_medium/V1_02_medium.zip",
      "vicon_room1/V1_03_difficult/V1_03_difficult.zip",
      "vicon_room2/V2_01_easy/V2_01_easy.zip",
      "vicon_room2/V2_02_medium/V2_02_medium.zip",
      "vicon_room2/V2_03_difficult/V2_03_difficult.zip"
  ]

  # Make destination folder if it doesn't exist already
  os.makedirs(dst_dir, exist_ok=True)

  # Download sequence
  for seq in seqs:
    src = os.path.join(base_url, seq)
    dst = os.path.join(dst_dir, os.path.basename(seq))
    download(src, dst)


def extract_zip(src, dst):
  """ Extract zip file """
  dir_name = os.path.basename(src).replace(".zip", "")
  dir_name = dir_name.replace("_easy", "")
  dir_name = dir_name.replace("_medium", "")
  dir_name = dir_name.replace("_difficult", "")
  dst_path = os.path.join(dst, dir_name)

  print(f"Extracting [{src}] -> [{dst_path}]", flush=True)
  with zipfile.ZipFile(src, 'r') as zip_ref:
    zip_ref.extractall(dst_path)


def extract_zips(src_dir, dst):
  """ Extract zip files """
  for zipfile in sorted(glob.glob(os.path.join(src_dir, "*.zip"))):
    extract_zip(zipfile, dst)


if __name__ == "__main__":
  download_dir = "/data/euroc/archive"
  dst_dir = "/data/euroc/"
  download_sequences(download_dir)
  extract_zips(download_dir, dst_dir)
