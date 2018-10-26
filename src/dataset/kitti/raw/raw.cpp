#include "prototype/dataset/kitti/raw/raw.hpp"

namespace prototype {

int kitti_raw_dataset_load(kitti_raw_dataset_t &ds) {
  // Pre-check
  bool res = dir_exists(ds.date_dir);
  CHECK(res == false, "Dataset path not found! [%s]", ds.date_dir.c_str());

  // Load calibration files
  int retval;
  // -- Camera to camera calibration
  ds.cam2cam = calib_cam2cam_t{ds.date_dir + "/calib_cam_to_cam.txt"};
  retval = calib_cam2cam_load(ds.cam2cam);
  CHECK(retval == 0, "Failed to load camera to camera calibration!");
  // -- IMU to Velo calibration
  ds.imu2velo = calib_imu2velo_t{ds.date_dir + "/calib_imu_to_velo.txt"};
  retval = calib_imu2velo_load(ds.imu2velo);
  CHECK(retval == 0, "Failed to load imu to velocalibration!");
  // -- Velo to camera calibration
  ds.velo2cam = calib_velo2cam_t{ds.date_dir + "/calib_velo_to_cam.txt"};
  retval = calib_velo2cam_load(ds.velo2cam);
  CHECK(retval == 0, "Failed to load velo to camera calibration!");

  // Load image files
  // -- Get list of image paths
  retval = list_dir(ds.drive_dir + "/image_00/data", ds.cam0);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_01/data", ds.cam1);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_02/data", ds.cam2);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_03/data", ds.cam3);
  CHECK(retval == 0, "Failed to traverse image files!");
  // -- Sort image paths
  std::sort(ds.cam0.begin(), ds.cam0.end());
  std::sort(ds.cam1.begin(), ds.cam1.end());
  std::sort(ds.cam2.begin(), ds.cam2.end());
  std::sort(ds.cam3.begin(), ds.cam3.end());
  // -- Add full path to files
  for (size_t i = 0; i < ds.cam0.size(); i++) {
    ds.cam0[i] = ds.drive_dir + "/image_00/data/" + ds.cam0[i];
    ds.cam1[i] = ds.drive_dir + "/image_01/data/" + ds.cam1[i];
    ds.cam2[i] = ds.drive_dir + "/image_02/data/" + ds.cam2[i];
    ds.cam3[i] = ds.drive_dir + "/image_03/data/" + ds.cam3[i];
  }

  // Load OXTS data
  ds.oxts = oxts_t{ds.drive_dir + "/oxts"};
  retval = oxts_load(ds.oxts);
  CHECK(retval == 0, "Failed to load OXTS data!");

  ds.ok = true;
  return 0;
error:
  return -1;
}

} //  namespace prototype
