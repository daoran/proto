#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include <vector>
#include <filesystem>

namespace cartesian {

namespace fs = std::filesystem;

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef KITTI_FATAL
#define KITTI_FATAL(...)                                                       \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[KITTI_FATAL] [%s:%d:%s()]: ",                                    \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

#ifndef KITTI_LOG
#define KITTI_LOG(...)                                                         \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[KITTI_LOG] [%s:%d:%s()]: ",                                      \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);
#endif

//////////////////
// KITTI Camera //
//////////////////

struct KittiCamera {
  std::vector<int64_t> timestamps;
  std::vector<fs::path> image_paths;

  KittiCamera() = delete;
  KittiCamera(const fs::path &data_dir);
  ~KittiCamera() = default;
};

////////////////
// KITTI OXTS //
////////////////

struct KittiOxts {
  // Timestamps
  std::vector<int64_t> timestamps;

  // GPS
  std::vector<double> lat; // Latitude [deg]
  std::vector<double> lon; // Longitude [deg]
  std::vector<double> alt; // Altitude [m]

  // Attitude
  std::vector<double> roll;  // Roll [rad]
  std::vector<double> pitch; // Pitch [rad]
  std::vector<double> yaw;   // Yaw [rad]

  // Velocity
  std::vector<double> vn; // Velocity towards north [m/s]
  std::vector<double> ve; // Velocity towards east [m/s]
  std::vector<double> vf; // Forward velocity [m/s]
  std::vector<double> vl; // Leftward velocity [m/s]
  std::vector<double> vu; // Upward velocity [m/s]

  // Acceleration
  std::vector<double> ax; // Acceleration in x [m/s^2]
  std::vector<double> ay; // Acceleration in y [m/s^2]
  std::vector<double> az; // Acceleration in z [m/s^2]
  std::vector<double> af; // Forward acceleration [m/s^2]
  std::vector<double> al; // Leftward acceleration [m/s^2]
  std::vector<double> au; // Upward acceleration [m/s^2]

  // Angular velocity
  std::vector<double> wx; // Angular rate around x [rad/s]
  std::vector<double> wy; // Angular rate around y [rad/s]
  std::vector<double> wz; // Angular rate around z [rad/s]
  std::vector<double> wf; // Angular rate around foward axis [rad/s]
  std::vector<double> wl; // Angular rate around left axis [rad/s]
  std::vector<double> wu; // Angular rate around up axis [rad/s]

  // Satellite tracking data
  std::vector<double> pos_accuracy; // Position accuracy [north / east in m]
  std::vector<double> vel_accuracy; // Velocity accuracy [north / east in m/s]
  std::vector<int> navstat;         // Navigation status
  std::vector<int> numsats;         // Number of satelllites tracked by GPS
  std::vector<int> posmode;         // Position mode
  std::vector<int> velmode;         // Velocity mode
  std::vector<int> orimode;         // Orientation mode

  KittiOxts() = delete;
  KittiOxts(const fs::path &data_dir);
  ~KittiOxts() = default;
};

////////////////////
// KITTI Velodyne //
////////////////////

struct KittiVelodyne {
  std::vector<int64_t> timestamps;
  std::vector<int64_t> timestamps_start;
  std::vector<int64_t> timestamps_end;
  std::vector<fs::path> pcd_paths;

  KittiVelodyne() = delete;
  KittiVelodyne(const fs::path &data_dir);
  ~KittiVelodyne() = default;

  static std::vector<std::array<float, 4>>
  load_points(const fs::path &pcd_path);
};

///////////////////////
// Kitti Calibration //
///////////////////////

struct KittiCalib {
  std::string calib_time_cam_to_cam;
  std::string calib_time_imu_to_velo;
  std::string calib_time_velo_to_cam;
  double corner_dist;

  double S_00[2];       // Image size [pixels]
  double K_00[9];       // Camera 0 intrinsics
  double D_00[5];       // Camera 0 distortion coefficients
  double R_00[9];       // Rotation from camera 0 to camera 0
  double T_00[3];       // Translation from camera 0 to camera 0
  double S_rect_00[2];  // Image size after rectifcation [pixels]
  double R_rect_00[9];  // Rotation after rectification
  double P_rect_00[12]; // Projection matrix after rectification

  double S_01[2];       // Image size [pixels]
  double K_01[9];       // Camera 1 intrinsics
  double D_01[5];       // Camera 1 distortion coefficients
  double R_01[9];       // Rotation from camera 0 to camera 1
  double T_01[3];       // Translation from camera 0 to camera 1
  double S_rect_01[2];  // Image size after rectifcation [pixels]
  double R_rect_01[9];  // Rotation after rectification
  double P_rect_01[12]; // Projection matrix after rectification

  double S_02[2];       // Image size [pixels]
  double K_02[9];       // Camera 2 intrinsics
  double D_02[5];       // Camera 2 distortion coefficients
  double R_02[9];       // Rotation from camera 0 to camera 2
  double T_02[3];       // Translation from camera 0 to camera 2
  double S_rect_02[2];  // Image size after rectifcation [pixels]
  double R_rect_02[9];  // Rotation after rectification
  double P_rect_02[12]; // Projection matrix after rectification

  double S_03[2];       // Image size [pixels]
  double K_03[9];       // Camera 3 intrinsics
  double D_03[5];       // Camera 3 distortion coefficients
  double R_03[9];       // Rotation from camera 0 to camera 3
  double T_03[3];       // Translation from camera 0 to camera 3
  double S_rect_03[2];  // Image size after rectifcation [pixels]
  double R_rect_03[9];  // Rotation after rectification
  double P_rect_03[12]; // Projection matrix after rectification

  double R_velo_imu[9]; // Rotation from imu to velodyne
  double T_velo_imu[3]; // Translation from imu to velodyne

  double R_cam_velo[9]; // Rotation from velodyne to camera
  double T_cam_velo[3]; // Translation from velodyne to camera
  double delta_f[2];
  double delta_c[2];

  KittiCalib() = delete;
  KittiCalib(const fs::path &data_dir);
  ~KittiCalib() = default;

  void print();
};

///////////////
// Kitti Raw //
///////////////

struct KittiRaw {
  std::string seq_name;
  std::shared_ptr<KittiCamera> image_00;
  std::shared_ptr<KittiCamera> image_01;
  std::shared_ptr<KittiCamera> image_02;
  std::shared_ptr<KittiCamera> image_03;
  std::shared_ptr<KittiOxts> oxts;
  std::shared_ptr<KittiVelodyne> velodyne;
  std::shared_ptr<KittiCalib> calib;

  KittiRaw() = delete;
  KittiRaw(const fs::path &data_dir, const std::string &seq_name);
  ~KittiRaw() = default;
};

} // namespace cartesian
