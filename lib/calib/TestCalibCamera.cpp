#include <gtest/gtest.h>

#include "calib/CalibCamera.hpp"
#include "sim/SimCalibCamera.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace xyz {

// TEST(CalibCamera, initializeIntrinsics) {
//   CalibCamera calib{TEST_CONFIG};
//   calib.initializeIntrinsics();
// }

// TEST(CalibCamera, initializeExtrinsic) {
//   CalibCamera calib{TEST_CONFIG};
//   calib.initializeIntrinsics();
//   calib.initializeExtrinsics();
// }

// TEST(CalibCamera, solve) {
//   CalibCamera calib{TEST_CONFIG};
//   calib.solve();
//   calib.saveResults("/tmp/calib_camera-results.yaml");
// }

TEST(CalibCamera, solve) {
  CalibCamera calib;
  SimCalibCamera sim;

  // Add cameras
  for (const auto &[_, camera] : sim.cameras) {
    calib.addCamera(camera.getCameraId(),
                    camera.getCameraModelString(),
                    camera.getResolution(),
                    camera.getIntrinsic(),
                    camera.getExtrinsic());
  }

  // Add target
  for (const auto &[_, config] : sim.target_configs) {
    calib.addTarget(config, tf_vec());
  }

  // Add camera data
  auto timeline = sim.get_timeline();
  for (const auto &ts: timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto camera_id = target_event->camera_id;
        const auto calib_target = target_event->calib_target;
        calib.addCameraMeasurement(ts, camera_id, calib_target);
      }
    }
  }

  calib.solve();
}

} // namespace xyz
