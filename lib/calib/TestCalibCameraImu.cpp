#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "calib/CalibCameraImu.hpp"
#include "sim/SimCalib.hpp"
#include "timeline/timeline.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera_imu.yaml"

namespace cartesian {

TEST(CalibCameraImu, construct) {

  // clang-format off
  Mat4 T_imu_cam0;
  T_imu_cam0 <<
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0;

  Mat4 T_imu_cam1;
  T_imu_cam1 <<
    0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
    0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
    -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
    0.0, 0.0, 0.0, 1.0;

  Mat4 T_cam0_cam1 = T_imu_cam0.inverse() * T_imu_cam1;
  print_matrix("T_cam0_cam1", T_cam0_cam1);
  print_vector("T_cam0_cam1", tf_vec(T_cam0_cam1));
  print_vector("T_cam0_imu", tf_vec(T_imu_cam0.inverse()));
  // clang-format on

  CalibCameraImu calib{TEST_CONFIG};
  auto cam0 = calib.getCameraGeometry(0);
  auto cam1 = calib.getCameraGeometry(1);
  auto imu0 = calib.getImuGeometry(0);

  double *cam0_int = cam0->intrinsic.data();
  cam0_int[0] = 458.654;
  cam0_int[1] = 457.296;
  cam0_int[2] = 367.215;
  cam0_int[3] = 248.375;
  cam0_int[4] = -0.28340811;
  cam0_int[5] = 0.07395907;
  cam0_int[6] = 0.00019359;
  cam0_int[7] = 1.76187114e-05;

  double *cam0_ext = cam0->extrinsic.data();
  cam0_ext[0] = 0.0;
  cam0_ext[1] = 0.0;
  cam0_ext[2] = 0.0;
  cam0_ext[3] = 0.0;
  cam0_ext[4] = 0.0;
  cam0_ext[5] = 0.0;
  cam0_ext[6] = 1.0;

  double *cam1_int = cam1->intrinsic.data();
  cam1_int[0] = 457.587;
  cam1_int[1] = 456.134;
  cam1_int[2] = 379.999;
  cam1_int[3] = 255.238;
  cam1_int[4] = -0.28368365;
  cam1_int[5] = 0.07451284;
  cam1_int[6] = -0.00010473;
  cam1_int[7] = -3.55590700e-05;

  double *cam1_ext = cam1->extrinsic.data();
  cam1_ext[0] = 0.109935;
  cam1_ext[1] = -0.000336639;
  cam1_ext[2] = 0.000643293;
  cam1_ext[3] = 0.00728166;
  cam1_ext[4] = -0.00287444;
  cam1_ext[5] = 0.00119855;
  cam1_ext[6] = 0.999969;

  // clang-format off
  Mat4 T_SC0;
  T_SC0 <<
    0.0148655429818, -0.999880929698,   0.00414029679422, -0.0216401454975,
    0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178,   0.00981073058949,
    0.0, 0.0, 0.0, 1.0;
    // 0.0148655429818, -0.999880929698,   0.00414029679422, -0.0,
    // 0.999557249008,   0.0149672133247,  0.025715529948,   -0.0,
    // -0.0257744366974, 0.00375618835797, 0.999660727178,   0.0,
    // 0.0, 0.0, 0.0, 1.0;
  Mat4 T_C0S = T_SC0.inverse();
  Vec7 imu_extrinsic = tf_vec(T_C0S);

  double *imu0_ext = imu0->extrinsic.data();
  imu0_ext[0] = imu_extrinsic[0];
  imu0_ext[1] = imu_extrinsic[1];
  imu0_ext[2] = imu_extrinsic[2];
  imu0_ext[3] = imu_extrinsic[3];
  imu0_ext[4] = imu_extrinsic[4];
  imu0_ext[5] = imu_extrinsic[5];
  imu0_ext[6] = imu_extrinsic[6];
  // clang-format on

  // Form timeline
  Timeline timeline;
  // -- Add camera data
  for (const auto &[camera_id, camera_data] : calib.getAllCameraData()) {
    for (const auto &[ts, target_map] : camera_data) {
      for (const auto &[target_id, target] : target_map) {
        timeline.add(ts, camera_id, target);
      }
    }
  }
  // -- Add imu data
  const int imu_index = 0;
  const auto imu_data = calib.getImuData(imu_index);
  for (int k = 0; k < imu_data.size(); k++) {
    const auto ts = imu_data.getTimestamp(k);
    const auto imu_acc = imu_data.getAcc(k);
    const auto imu_gyr = imu_data.getGyr(k);
    timeline.add(ts, imu_acc, imu_gyr);
  }

  // Iterate through timeline
  for (const auto ts : timeline.timestamps) {
    // Handle multiple events in the same timestamp
    for (auto &event : timeline.getEvents(ts)) {
      // Camera event
      if (auto camera_event = dynamic_cast<CalibTargetEvent *>(event)) {
        calib.addMeasurement(ts,
                             camera_event->camera_id,
                             camera_event->calib_target);
      }

      // Imu Event
      if (const auto &imu_event = dynamic_cast<ImuEvent *>(event)) {
        calib.addMeasurement(ts, imu_event->acc, imu_event->gyr);
      }
    }

    // if (calib.poses.size() >= 2) {
    //   break;
    // }

    // Solve
    // calib.solve();
  }

  // Solve
  calib.solve();

  // // Log
  // {
  //   const auto target_config = calib.target_configs[0];
  //   const Mat4 T_WT0 = tf(calib.target_pose);
  //
  //   std::map<timestamp_t, Mat4> sensor_poses;
  //   std::map<timestamp_t, Mat4> cam0_poses;
  //   std::map<timestamp_t, Mat4> cam1_poses;
  //   const Mat4 T_SC0 = tf(calib.imu_geometries[0]->extrinsic).inverse();
  //   const Mat4 T_C0C1 = tf(calib.camera_geometries[1]->extrinsic);
  //   for (const auto &[ts, pose] : calib.poses) {
  //     const Mat4 T_WS = tf(pose);
  //     sensor_poses[ts] = T_WS;
  //     cam0_poses[ts] = T_WS * T_SC0;
  //     cam1_poses[ts] = T_WS * T_SC0 * T_C0C1;
  //   }
  //
  //   const std::string cam0_dir = "/data/euroc/imu_april/mav0/cam0/data";
  //   std::map<timestamp_t, fs::path> cam0_image_paths;
  //   for (const auto &entry : fs::directory_iterator(cam0_dir)) {
  //     const auto image_path = entry.path();
  //     const timestamp_t ts = stol(image_path.stem().string().c_str());
  //     cam0_image_paths[ts] = image_path;
  //   }
  //
  //   const std::string cam1_dir = "/data/euroc/imu_april/mav0/cam0/data";
  //   std::map<timestamp_t, fs::path> cam1_image_paths;
  //   for (const auto &entry : fs::directory_iterator(cam1_dir)) {
  //     const auto image_path = entry.path();
  //     const timestamp_t ts = stol(image_path.stem().string().c_str());
  //     cam1_image_paths[ts] = image_path;
  //   }
  //
  //   Logger log;
  //
  //   const Vec3 red{255.0, 0.0, 0.0};
  //   const Vec3 green{0.0, 255.0, 0.0};
  //   const Vec3 blue{0.0, 0.0, 255.0};
  //
  //   log.log_poses("sensor_poses", sensor_poses, 0.1);
  //   log.log_poses("cam0_poses", cam0_poses, 0.1);
  //   log.log_poses("cam1_poses", cam1_poses, 0.1);
  //   log.log_target("calib_target", target_config, T_WT0);
  //
  //   log.init_series_line("imu/acc/x", red, 1.0f);
  //   log.init_series_line("imu/acc/y", green, 1.0f);
  //   log.init_series_line("imu/acc/z", blue, 1.0f);
  //   log.init_series_line("imu/gyr/x", red, 1.0f);
  //   log.init_series_line("imu/gyr/y", green, 1.0f);
  //   log.init_series_line("imu/gyr/z", blue, 1.0f);
  //
  //   log.init_series_line("imu_error/acc/x", red, 1.0f);
  //   log.init_series_line("imu_error/acc/y", green, 1.0f);
  //   log.init_series_line("imu_error/acc/z", blue, 1.0f);
  //   log.init_series_line("imu_error/gyr/x", red, 1.0f);
  //   log.init_series_line("imu_error/gyr/y", green, 1.0f);
  //   log.init_series_line("imu_error/gyr/z", blue, 1.0f);
  //
  //   for (size_t k = 0; k < calib.imu_data[0].timestamps.size(); ++k) {
  //     const timestamp_t ts = calib.imu_data[0].timestamps[k];
  //     const Vec3 &acc = calib.imu_data[0].acc_data[k];
  //     const Vec3 &gyr = calib.imu_data[0].gyr_data[k];
  //
  //     log.log_scalar("imu/acc/x", ts, acc.x());
  //     log.log_scalar("imu/acc/y", ts, acc.y());
  //     log.log_scalar("imu/acc/z", ts, acc.z());
  //     log.log_scalar("imu/gyr/x", ts, gyr.x());
  //     log.log_scalar("imu/gyr/y", ts, gyr.y());
  //     log.log_scalar("imu/gyr/z", ts, gyr.z());
  //   }
  //
  //   for (auto &[ts, factors] : calib.imu_resblocks) {
  //     const auto &imu_buffer = factors[0]->imu_buffer_;
  //
  //     for (size_t k = 0; k < imu_buffer.timestamps.size(); ++k) {
  //       const timestamp_t ts = imu_buffer.timestamps[k];
  //       const Vec3 &acc = imu_buffer.acc_data[k];
  //       const Vec3 &gyr = imu_buffer.gyr_data[k];
  //
  //       log.log_scalar("imu_error/acc/x", ts, acc.x());
  //       log.log_scalar("imu_error/acc/y", ts, acc.y());
  //       log.log_scalar("imu_error/acc/z", ts, acc.z());
  //       log.log_scalar("imu_error/gyr/x", ts, gyr.x());
  //       log.log_scalar("imu_error/gyr/y", ts, gyr.y());
  //       log.log_scalar("imu_error/gyr/z", ts, gyr.z());
  //     }
  //   }
  //
  //   // for (const auto &[ts, image_path] : cam0_image_paths) {
  //   //   const auto image = cv::imread(image_path.string());
  //   //   log.log_image("cam0", ts, image);
  //   // }
  //   // for (const auto &[ts, image_path] : cam1_image_paths) {
  //   //   const auto image = cv::imread(image_path.string());
  //   //   log.log_image("cam1", ts, image);
  //   // }
  // }
}

// TEST(CalibCameraImu, construct) {
//   // Simulate
//   SimCalib sim;
//   sim.sim_camimu_calib();
//
//   // Cslibrate
//   CalibCameraImu calib;
//
//   // -- Add target
//   calib.addTarget(sim.target_configs[0], sim.targets.at(0).extrinsic);
//
//   // -- Add camera
//   for (auto &[camera_id, camera] : sim.cameras) {
//     calib.addCamera(camera.camera_id,
//                     camera.camera_model->type(),
//                     camera.resolution,
//                     camera.intrinsic,
//                     camera.extrinsic);
//   }
//
//   // -- Add Imu
//   for (auto &[imu_id, imu] : sim.imus) {
//     calib.addImu(imu_id, imu.imu_params, imu.extrinsic);
//   }
//
//   // -- Add data
//   auto timeline = sim.get_timeline();
//   for (const auto &ts : timeline.timestamps) {
//     for (const auto &event : timeline.getEvents(ts)) {
//       if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
//         calib.addMeasurement(ts,
//                              target_event->camera_id,
//                              target_event->calib_target);
//       }
//
//       if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
//         calib.addMeasurement(ts, imu_event->acc, imu_event->gyr);
//       }
//     }
//   }
//
//   // clang-format off
//   Mat4 T_C0S;
//   T_C0S <<
//     0.0, 1.0, 0.0, 0.1,
//     1.0, 0.0, 0.0, 0.2,
//     0.0, 0.0, 1.0, 0.3,
//     0.0, 0.0, 0.0, 1.0;
//   Vec7 imu_extrinsic = tf_vec(T_C0S);
//
//   double *imu0_ext = calib.imu_geometries.at(0)->extrinsic.data();
//   imu0_ext[0] = imu_extrinsic[0];
//   imu0_ext[1] = imu_extrinsic[1];
//   imu0_ext[2] = imu_extrinsic[2];
//   imu0_ext[3] = imu_extrinsic[3];
//   imu0_ext[4] = imu_extrinsic[4];
//   imu0_ext[5] = imu_extrinsic[5];
//   imu0_ext[6] = imu_extrinsic[6];
//   // clang-format on
//
//   calib.solve();
// }

} // namespace cartesian
