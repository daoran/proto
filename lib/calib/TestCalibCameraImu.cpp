#include <gtest/gtest.h>

#include "calib/CalibCameraImu.hpp"
#include "timeline/timeline.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera_imu.yaml"

namespace xyz {

TEST(CalibCameraImu, construct) {
  CalibCameraImu calib{TEST_CONFIG};

  // Form timeline
  Timeline timeline;

  // -- Add camera data
  for (const auto &[camera_index, measurements] : calib.getAllCameraData()) {
    for (const auto &[ts, calib_target] : measurements) {
      timeline.add(ts, camera_index, calib_target);
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
      if (auto camera_event = static_cast<CalibTargetEvent *>(event)) {
        const int camera_index = camera_event->camera_index;
        const auto calib_target = camera_event->calib_target;
        calib.addMeasurement(camera_index, calib_target);
      }

      // Imu Event
      if (const auto &imu_event = static_cast<ImuEvent *>(event)) {
        calib.addMeasurement(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }
}

} // namespace xyz
