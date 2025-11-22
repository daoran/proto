#include <gtest/gtest.h>

#include "calib/CalibData.hpp"
#include "ceres/CalibCameraError.hpp"

namespace xyz {

static Vec7 setup_body_pose() {
  // Body pose in world frame: T_WB
  // clang-format off
  Mat4 T_WB;
  T_WB <<  1.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;
  T_WB = tf_perturb_rot(T_WB, 0.01, 1);
  auto body_pose = tf_vec(T_WB);
  // ^ Note: Due to numerical stability issues the translation component
  // cannot be 0 for checking jacobians
  // clang-format on

  return body_pose;
}

static Vec7 setup_fiducial_pose() {
  // Fiducial pose: T_WF
  // clang-format off
  Mat4 T_WF;
  T_WF <<  0.0,  0.0, -1.0, 0.5,
          -1.0,  0.0,  0.0, 0.001,
           0.0,  1.0,  0.0, 0.001,
           0.0,  0.0,  0.0, 1.0;
  T_WF = tf_perturb_rot(T_WF, 0.01, 2);
  auto fiducial_pose = tf_vec(T_WF);
  // clang-format on

  return fiducial_pose;
}

static Vec7 setup_camera_extrinsic() {
  // Camera extrinsics: T_BCi
  // clang-format off
  Mat4 T_BCi;
  T_BCi << 1.0, 0.0, 0.0, -0.01,
           0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;
  T_BCi = tf_perturb_rot(T_BCi, 0.01, 2);
  auto camera_extrinsic = tf_vec(T_BCi);
  // clang-format on

  return camera_extrinsic;
}

static std::shared_ptr<CameraGeometry>
setup_camera_geometry(const int camera_index, const Vec7 &extrinsic) {
  const Vec2i resolution{752, 480};
  const std::string camera_model = "BrownConrady4";

  // Intrinsic
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;

  VecX intrinsic;
  intrinsic.resize(8);
  intrinsic << fx, fy, cx, cy, k1, k2, p1, p2;

  return std::make_shared<CameraGeometry>(camera_index,
                                          camera_model,
                                          resolution,
                                          intrinsic,
                                          extrinsic);
}

static AprilGrid setup_aprilgrid() {
  const timestamp_t ts = 0;
  const int tag_rows = 10;
  const int tag_cols = 10;
  const double tag_size = 0.08;
  const double tag_spacing = 0.3;
  AprilGrid aprilgrid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

  return aprilgrid;
}

void simulate_camera_measurements(
    const AprilGrid &aprilgrid,
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const Vec7 body_pose,
    const Vec7 fiducial_pose,
    std::vector<int> &tag_ids,
    std::vector<int> &corner_indicies,
    Vec2s &keypoints,
    Vec3s &object_points) {
  // Get camera parameters
  const auto res = camera_geometry->getResolution();
  const auto camera = camera_geometry->getCameraModel();
  const auto params = camera_geometry->getIntrinsic();
  const Vec7 camera_extrinsic = camera_geometry->getExtrinsic();

  // Form fiducial target to camera pose T_CiF
  const Mat4 T_WB = tf(body_pose);
  const Mat4 T_WF = tf(fiducial_pose);
  const Mat4 T_BCi = tf(camera_extrinsic);
  const Mat4 T_CiF = T_BCi.inverse() * T_WB.inverse() * T_WF;

  // Simulate camera measurements
  const int tag_rows = aprilgrid.getTagRows();
  const int tag_cols = aprilgrid.getTagCols();
  for (int tag_id = 0; tag_id < (tag_rows * tag_cols); ++tag_id) {
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      const Vec3 p_F = aprilgrid.getObjectPoint(tag_id, corner_index);
      const Vec3 p_C = tf_point(T_CiF, p_F);
      Vec2 z{0.0, 0.0};
      if (camera->project(res, params, p_C, z) != 0) {
        continue;
      }

      tag_ids.push_back(tag_id);
      corner_indicies.push_back(corner_index);
      object_points.push_back(p_F);
      keypoints.push_back(z);
    }
  }
}

TEST(CalibCameraError, evaluate) {
  // Setup
  auto body_pose = setup_body_pose();
  auto fiducial_pose = setup_fiducial_pose();
  auto camera_extrinsic = setup_camera_extrinsic();
  auto camera_geometry = setup_camera_geometry(0, camera_extrinsic);
  auto aprilgrid = setup_aprilgrid();

  // Simulate camera measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  simulate_camera_measurements(aprilgrid,
                               camera_geometry,
                               body_pose,
                               fiducial_pose,
                               tag_ids,
                               corner_indicies,
                               keypoints,
                               object_points);

  // Create residual block
  const Mat2 covar = I(2);
  const Mat4 T_WB = tf(body_pose);
  const Mat4 T_WF = tf(fiducial_pose);
  const Mat4 T_BF = T_WB.inverse() * T_WF;
  VecX relpose = tf_vec(T_BF);
  auto resblock = CalibCameraError::create(camera_geometry,
                                           relpose.data(),
                                           object_points[0].data(),
                                           keypoints[0],
                                           covar);

  // Check residual size and parameter block sizes
  auto block_sizes = resblock->parameter_block_sizes();
  ASSERT_EQ(resblock->num_residuals(), 2);
  ASSERT_EQ(block_sizes[0], 3);
  ASSERT_EQ(block_sizes[1], 7);
  ASSERT_EQ(block_sizes[2], 7);
  ASSERT_EQ(block_sizes[3], 8);

  // Check param pointers
  auto param_ptrs = resblock->getParamPtrs();
  ASSERT_EQ(param_ptrs[0], object_points[0].data());
  ASSERT_EQ(param_ptrs[1], relpose.data());
  ASSERT_EQ(param_ptrs[2], camera_geometry->getExtrinsicPtr());
  ASSERT_EQ(param_ptrs[3], camera_geometry->getIntrinsicPtr());

  // Check Jacobians
  ASSERT_TRUE(resblock->checkJacobian(0, "J_point"));
  ASSERT_TRUE(resblock->checkJacobian(1, "J_relpose"));
  ASSERT_TRUE(resblock->checkJacobian(2, "J_extrinsic"));
  ASSERT_TRUE(resblock->checkJacobian(3, "J_camera"));
}

} // namespace xyz
