#ifndef PROTOTYPE_DATASET_KITTI_RAW_HPP
#define PROTOTYPE_DATASET_KITTI_RAW_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "prototype/core/core.hpp"
#include "prototype/dataset/kitti/raw/calib.hpp"
#include "prototype/dataset/kitti/raw/oxts.hpp"
#include "prototype/dataset/kitti/raw/parse.hpp"

namespace prototype {

/**
 * KITTI Raw Dataset
 */
struct kitti_raw_dataset_t {
  bool ok = false;

  std::string raw_dir;
  std::string date;
  std::string seq;
  std::string date_dir;
  std::string drive_dir;

  calib_cam2cam_t cam2cam;
  calib_imu2velo_t imu2velo;
  calib_velo2cam_t velo2cam;

  oxts_t oxts;
  std::vector<std::string> cam0;
  std::vector<std::string> cam1;
  std::vector<std::string> cam2;
  std::vector<std::string> cam3;

  kitti_raw_dataset_t() {}

  kitti_raw_dataset_t(const std::string &raw_dir,
                      const std::string &date,
                      const std::string &seq)
      : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
        date_dir{raw_dir + "/" + date}, drive_dir{date_dir + "/" + date +
                                                  "_drive_" + seq + "_sync"} {}

  kitti_raw_dataset_t(const std::string &raw_dir,
                      const std::string &date,
                      const std::string &seq,
                      const std::string &type)
      : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
        date_dir{raw_dir + "/" + date}, drive_dir{date_dir + "/" + date +
                                                  "_drive_" + seq + "_" +
                                                  type} {}
};

/**
 * Load KITTI raw dataset
 *
 * @param[in,out] dataset KITTI raw dataset
 * @returns 0 or -1 for success or failure
 */
int kitti_raw_dataset_load(kitti_raw_dataset_t &dataset);

} // namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_RAW_HPP
