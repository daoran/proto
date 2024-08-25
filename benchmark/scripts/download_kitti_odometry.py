#!/usr/bin/env python3
import os
import glob
import urllib.request
import zipfile

from tqdm import tqdm


def download(url, save_path):
  # Get the size of the file first
  response = urllib.request.urlopen(url)
  total_size = int(response.info().get('Content-Length').strip())

  # Initialize the progress bar
  pbar = tqdm(total=total_size, unit='B', unit_scale=True, desc=save_path)

  try:
    # Open the URL and the output file
    response = urllib.request.urlopen(url)
    out_file = open(save_path, 'wb')

    # Read and write the file in chunks
    chunk_size = 1024
    while True:
      chunk = response.read(chunk_size)
      if not chunk:
        break
      out_file.write(chunk)
      pbar.update(len(chunk))

  finally:
    # Clean up: Close the file and the response
    pbar.close()
    out_file.close()
    response.close()


def extract_zip(src, dst):
  """ Extract zip file """
  print(f"Extracting [{src}] -> [{dst}]", flush=True)
  with zipfile.ZipFile(src, 'r') as zip_ref:
    zip_ref.extractall(dst)


def download_kitti_odometry(download_path):
  """ Download KITTI Odometry Dataset """
  kitti_url = "https://s3.eu-central-1.amazonaws.com/avg-kitti"
  sequences = [
      "data_odometry_calib.zip", "data_odometry_gray.zip",
      "data_odometry_poses.zip"
  ]

  os.makedirs(download_path, exist_ok=True)
  for seq in sequences:
    url = f"{kitti_url}/{seq}"
    url_dst = f"{download_path}/{seq}"
    download(url, url_dst)
    extract_zip(url_dst, download_path)


if __name__ == "__main__":
  download_path = "/data/kitti_odometry"
  download_kitti_odometry(download_path)
