#include <gtest/gtest.h>

#include "calib/CalibCamera.hpp"
#include "calib/CalibCameraImu.hpp"
#include "timeline/timeline.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera_imu.yaml"

namespace cartesian {

TEST(CalibCameraImu, construct) {
  // CalibCamera calib{TEST_CONFIG};
  // calib.solve();

  CalibCameraImu calib{TEST_CONFIG};

  auto cam0 = calib.getCameraGeometry(0);
  auto cam1 = calib.getCameraGeometry(1);
  auto imu0 = calib.getImuGeometry(0);

  double *cam0_int = cam0->intrinsic.data();
  cam0_int[0] = 458.703;
  cam0_int[1] = 457.581;
  cam0_int[2] = 367.901;
  cam0_int[3] = 245.616;
  cam0_int[4] = -0.273273;
  cam0_int[5] = 0.0655293;
  cam0_int[6] = 0.000635649;
  cam0_int[7] = -0.000204924;

  double *cam0_ext = cam0->extrinsic.data();
  cam0_ext[0] = 0.0;
  cam0_ext[1] = 0.0;
  cam0_ext[2] = 0.0;
  cam0_ext[3] = 0.0;
  cam0_ext[4] = 0.0;
  cam0_ext[5] = 0.0;
  cam0_ext[6] = 1.0;

  double *cam1_int = cam1->intrinsic.data();
  cam1_int[0] = 457.441;
  cam1_int[1] = 456.264;
  cam1_int[2] = 378.15;
  cam1_int[3] = 252.206;
  cam1_int[4] = -0.270561;
  cam1_int[5] = 0.0632106;
  cam1_int[6] = 0.000426177;
  cam1_int[7] = -0.000205956;

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
  Mat4 T_C0S = T_SC0.inverse();
  Vec7 imu_extrinsic = tf_vec(T_C0S);
  print_vector("imu extrinsic", imu_extrinsic);

  // double *imu0_ext = imu0->extrinsic.data();
  // imu0_ext[0] = imu_extrinsic[0];
  // imu0_ext[1] = imu_extrinsic[1];
  // imu0_ext[2] = imu_extrinsic[2];
  // imu0_ext[3] = imu_extrinsic[3];
  // imu0_ext[4] = imu_extrinsic[4];
  // imu0_ext[5] = imu_extrinsic[5];
  // imu0_ext[6] = imu_extrinsic[6];
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
  for (int k = 0; k < imu_data.getNumMeasurements(); k++) {
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
        const int camera_id = camera_event->camera_id;
        const auto calib_target = camera_event->calib_target;
        calib.addMeasurement(ts, camera_id, calib_target);
      }

      // Imu Event
      if (const auto &imu_event = dynamic_cast<ImuEvent *>(event)) {
        calib.addMeasurement(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  // Solve
  calib.solve();
}

} // namespace cartesian
