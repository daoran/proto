#!/usr/bin/env python3
import os
import glob
import urllib.request
import zipfile

from tqdm import tqdm

################################################################################
# DOWNLOAD UTILS
################################################################################


def download(url, save_dir):
  # Get the size of the file first
  response = urllib.request.urlopen(url)
  total_size = int(response.info().get('Content-Length').strip())

  # Initialize the progress bar
  desc = f"Downloading [{url}]"
  pbar = tqdm(total=total_size, unit='B', unit_scale=True, desc=desc)

  try:
    # Open the URL and the output file
    response = urllib.request.urlopen(url)
    save_path = os.path.join(save_dir, os.path.basename(url))
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
  dir_name = os.path.basename(src).replace(".zip", "")
  dir_name = dir_name.replace("_easy", "")
  dir_name = dir_name.replace("_medium", "")
  dir_name = dir_name.replace("_difficult", "")
  dst_path = os.path.join(dst, dir_name)

  with zipfile.ZipFile(src, 'r') as zip_ref:
    zip_ref.extractall(dst_path)


################################################################################
# EUROC DATASET
################################################################################


def download_euroc_sequences(dst_dir):
  """ Download EuRoC MAV sequences """
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
      "vicon_room2/V2_03_difficult/V2_03_difficult.zip",
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "euroc2")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequences
  for seq in seqs:
    download(os.path.join(base_url, seq), ds_path)

    seq_name = os.path.basename(seq).replace(".zip", "")
    seq_name = seq_name.replace("_easy", "")
    seq_name = seq_name.replace("_medium", "")
    seq_name = seq_name.replace("_difficult", "")
    seq_dir = os.path.join(ds_path, seq_name)
    os.makedirs(seq_dir, exist_ok=True)

    zip_file = os.path.join(ds_path, os.path.basename(seq))
    extract_zip(zip_file, seq_dir)


def download_euroc_rosbags(dst_dir):
  """ Download EuRoC MAV ROS bags """
  base_url = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
  bags = [
      "machine_hall/MH_01_easy/MH_01_easy.bag",
      "machine_hall/MH_02_easy/MH_02_easy.bag",
      "machine_hall/MH_03_medium/MH_03_medium.bag",
      "machine_hall/MH_04_difficult/MH_04_difficult.bag",
      "machine_hall/MH_05_difficult/MH_05_difficult.bag",
      "machine_hall/MH_01_easy/MH_01_easy.bag",
      "machine_hall/MH_02_easy/MH_02_easy.bag",
      "machine_hall/MH_03_medium/MH_03_medium.bag",
      "machine_hall/MH_04_difficult/MH_04_difficult.bag",
      "machine_hall/MH_05_difficult/MH_05_difficult.bag",
      "vicon_room1/V1_01_easy/V1_01_easy.bag",
      "vicon_room1/V1_02_medium/V1_02_medium.bag",
      "vicon_room1/V1_03_difficult/V1_03_difficult.bag",
      "vicon_room2/V2_01_easy/V2_01_easy.bag",
      "vicon_room2/V2_02_medium/V2_02_medium.bag",
      "vicon_room2/V2_03_difficult/V2_03_difficult.bag",
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "euroc")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequences
  for bag in bags:
    download(os.path.join(base_url, bag), ds_path)

    old_bag_path = os.path.join(ds_path, bag_name)
    new_bag_path = old_bag_path.replace("_easy", "")
    new_bag_path = new_bag_path.replace("_medium", "")
    new_bag_path = new_bag_path.replace("_difficult", "")
    os.rename(old_bag_path, new_bag_path)

################################################################################
# KITTI ODOMETRY DATASET
################################################################################


def download_kitti_odometry(dst_dir):
  """ Download KITTI Odometry Dataset """
  kitti_url = "https://s3.eu-central-1.amazonaws.com/avg-kitti"
  sequences = [
      "data_odometry_calib.zip", "data_odometry_gray.zip",
      "data_odometry_poses.zip"
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "kitti_odometry")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequence
  for seq in sequences:
    url = f"{kitti_url}/{seq}"
    url_dst = f"{ds_path}/{seq}"
    download(url, url_dst)
    extract_zip(url_dst, ds_path)


################################################################################
# KITTI RAW DATASET
################################################################################


def download_kitti_raw(dst_dir):
  # download_path = "/data/kitt_raw"
  kitti_url = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
  sequences = [
      "2011_09_26_calib.zip", "2011_09_26_drive_0001", "2011_09_26_drive_0002",
      "2011_09_26_drive_0005", "2011_09_26_drive_0009", "2011_09_26_drive_0011",
      "2011_09_26_drive_0013", "2011_09_26_drive_0014", "2011_09_26_drive_0015",
      "2011_09_26_drive_0017", "2011_09_26_drive_0018", "2011_09_26_drive_0019",
      "2011_09_26_drive_0020", "2011_09_26_drive_0022", "2011_09_26_drive_0023",
      "2011_09_26_drive_0027", "2011_09_26_drive_0028", "2011_09_26_drive_0029",
      "2011_09_26_drive_0032", "2011_09_26_drive_0035", "2011_09_26_drive_0036",
      "2011_09_26_drive_0039", "2011_09_26_drive_0046", "2011_09_26_drive_0048",
      "2011_09_26_drive_0051", "2011_09_26_drive_0052", "2011_09_26_drive_0056",
      "2011_09_26_drive_0057", "2011_09_26_drive_0059", "2011_09_26_drive_0060",
      "2011_09_26_drive_0061", "2011_09_26_drive_0064", "2011_09_26_drive_0070",
      "2011_09_26_drive_0079", "2011_09_26_drive_0084", "2011_09_26_drive_0086",
      "2011_09_26_drive_0087", "2011_09_26_drive_0091", "2011_09_26_drive_0093",
      "2011_09_26_drive_0095", "2011_09_26_drive_0096", "2011_09_26_drive_0101",
      "2011_09_26_drive_0104", "2011_09_26_drive_0106", "2011_09_26_drive_0113",
      "2011_09_26_drive_0117", "2011_09_26_drive_0119", "2011_09_28_calib.zip",
      "2011_09_28_drive_0001", "2011_09_28_drive_0002", "2011_09_28_drive_0016",
      "2011_09_28_drive_0021", "2011_09_28_drive_0034", "2011_09_28_drive_0035",
      "2011_09_28_drive_0037", "2011_09_28_drive_0038", "2011_09_28_drive_0039",
      "2011_09_28_drive_0043", "2011_09_28_drive_0045", "2011_09_28_drive_0047",
      "2011_09_28_drive_0053", "2011_09_28_drive_0054", "2011_09_28_drive_0057",
      "2011_09_28_drive_0065", "2011_09_28_drive_0066", "2011_09_28_drive_0068",
      "2011_09_28_drive_0070", "2011_09_28_drive_0071", "2011_09_28_drive_0075",
      "2011_09_28_drive_0077", "2011_09_28_drive_0078", "2011_09_28_drive_0080",
      "2011_09_28_drive_0082", "2011_09_28_drive_0086", "2011_09_28_drive_0087",
      "2011_09_28_drive_0089", "2011_09_28_drive_0090", "2011_09_28_drive_0094",
      "2011_09_28_drive_0095", "2011_09_28_drive_0096", "2011_09_28_drive_0098",
      "2011_09_28_drive_0100", "2011_09_28_drive_0102", "2011_09_28_drive_0103",
      "2011_09_28_drive_0104", "2011_09_28_drive_0106", "2011_09_28_drive_0108",
      "2011_09_28_drive_0110", "2011_09_28_drive_0113", "2011_09_28_drive_0117",
      "2011_09_28_drive_0119", "2011_09_28_drive_0121", "2011_09_28_drive_0122",
      "2011_09_28_drive_0125", "2011_09_28_drive_0126", "2011_09_28_drive_0128",
      "2011_09_28_drive_0132", "2011_09_28_drive_0134", "2011_09_28_drive_0135",
      "2011_09_28_drive_0136", "2011_09_28_drive_0138", "2011_09_28_drive_0141",
      "2011_09_28_drive_0143", "2011_09_28_drive_0145", "2011_09_28_drive_0146",
      "2011_09_28_drive_0149", "2011_09_28_drive_0153", "2011_09_28_drive_0154",
      "2011_09_28_drive_0155", "2011_09_28_drive_0156", "2011_09_28_drive_0160",
      "2011_09_28_drive_0161", "2011_09_28_drive_0162", "2011_09_28_drive_0165",
      "2011_09_28_drive_0166", "2011_09_28_drive_0167", "2011_09_28_drive_0168",
      "2011_09_28_drive_0171", "2011_09_28_drive_0174", "2011_09_28_drive_0177",
      "2011_09_28_drive_0179", "2011_09_28_drive_0183", "2011_09_28_drive_0184",
      "2011_09_28_drive_0185", "2011_09_28_drive_0186", "2011_09_28_drive_0187",
      "2011_09_28_drive_0191", "2011_09_28_drive_0192", "2011_09_28_drive_0195",
      "2011_09_28_drive_0198", "2011_09_28_drive_0199", "2011_09_28_drive_0201",
      "2011_09_28_drive_0204", "2011_09_28_drive_0205", "2011_09_28_drive_0208",
      "2011_09_28_drive_0209", "2011_09_28_drive_0214", "2011_09_28_drive_0216",
      "2011_09_28_drive_0220", "2011_09_28_drive_0222", "2011_09_28_drive_0225",
      "2011_09_29_calib.zip", "2011_09_29_drive_0004", "2011_09_29_drive_0026",
      "2011_09_29_drive_0071", "2011_09_29_drive_0108", "2011_09_30_calib.zip",
      "2011_09_30_drive_0016", "2011_09_30_drive_0018", "2011_09_30_drive_0020",
      "2011_09_30_drive_0027", "2011_09_30_drive_0028", "2011_09_30_drive_0033",
      "2011_09_30_drive_0034", "2011_09_30_drive_0072", "2011_10_03_calib.zip",
      "2011_10_03_drive_0027", "2011_10_03_drive_0034", "2011_10_03_drive_0042",
      "2011_10_03_drive_0047", "2011_10_03_drive_0058"
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "kitti_raw")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequences
  for seq in sequences:
    url_path = seq if seq[-4:] == ".zip" else f"{seq}/{seq}_sync.zip"
    save_path = f"{ds_path}/{url_path}"
    # os.makedirs(os.path.dirname(save_path), exist_ok=True)

    download(f"{kitti_url}/{url_path}", ds_path)
    # extract_zip(save_path, os.path.dirname(save_path))


if __name__ == "__main__":
  dst_dir = "/data"
  download_euroc_sequences(dst_dir)
  # download_euroc_rosbags(dst_dir)
  # download_kitti_odometry(dst_dir)
  # download_kitti_raw(dst_dir)
  pass
