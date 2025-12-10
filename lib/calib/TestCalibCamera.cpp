#include <gtest/gtest.h>

#include "calib/CalibCamera.hpp"
#include "sim/SimCalibCamera.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace xyz {

static void perturb_camera_intrinsic(CameraGeometryPtr &camera) {
  auto intrinsic = camera->getIntrinsicPtr();
  intrinsic[0] += randf(0, 100);
  intrinsic[1] += randf(0, 100);
  intrinsic[2] += randf(0, 100);
  intrinsic[3] += randf(0, 100);
  intrinsic[4] = 0.0;
  intrinsic[5] = 0.0;
  intrinsic[6] = 0.0;
  intrinsic[7] = 0.0;
}

static void perturb_camera_extrinsic(CameraGeometryPtr &camera) {
  const auto phi = randf(-1.0, 1.0);
  const auto theta = randf(-1.0, 1.0);
  const auto psi = randf(-1.0, 1.0);
  const auto quat = euler2quat(Vec3{phi, theta, psi});

  auto extrinsic = camera->getExtrinsicPtr();
  extrinsic[0] = randf(0, 0.5);
  extrinsic[1] = randf(0, 0.5);
  extrinsic[2] = randf(0, 0.5);
  extrinsic[3] = quat.x();
  extrinsic[4] = quat.y();
  extrinsic[5] = quat.z();
  extrinsic[6] = quat.w();
}

static void perturb_camera(CameraGeometryPtr &camera) {
  perturb_camera_intrinsic(camera);
  if (camera->getCameraId() != 0) {
    perturb_camera_extrinsic(camera);
  }
}

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
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto camera_id = target_event->camera_id;
        const auto calib_target = target_event->calib_target;
        calib.addCameraMeasurement(ts, camera_id, calib_target);
      }
    }
  }

  // Perturb camera intrinsic and extrinsic
  perturb_camera(calib.getCameraGeometry(0));
  perturb_camera(calib.getCameraGeometry(1));

  // Solve
  calib.solve();
}

} // namespace xyz
