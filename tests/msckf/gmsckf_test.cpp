#include "dataset/euroc/mav_dataset.hpp"
#include "dataset/kitti/kitti.hpp"
#include "feature2d/klt_tracker.hpp"
#include "feature2d/orb_tracker.hpp"
#include "msckf/blackbox.hpp"
#include "msckf/gmsckf.hpp"
#include "prototype/core.hpp"
#include "prototype/munit.hpp"
#include "sim/world.hpp"

namespace prototype {

// clang-format off
static const std::string KITTI_RAW_DATASET = "/data/kitti/raw";
static const std::string TEST_CONFIG = "test_configs/msckf/gmsckf.yaml";
// clang-format on

int test_GMSCKF_constructor() {
  GMSCKF gmsckf;

  MU_CHECK_EQ(15, gmsckf.P_imu.rows());
  MU_CHECK_EQ(15, gmsckf.P_imu.cols());
  MU_CHECK_EQ(6, gmsckf.P_cam.rows());
  MU_CHECK_EQ(6, gmsckf.P_cam.cols());
  MU_CHECK_EQ(15, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(6, gmsckf.P_imu_cam.cols());
  MU_CHECK_EQ(12, gmsckf.Q_imu.rows());
  MU_CHECK_EQ(12, gmsckf.Q_imu.cols());

  MU_CHECK_EQ(0, gmsckf.counter_frame_id);
  MU_CHECK(zeros(3, 1).isApprox(gmsckf.p_IC));
  MU_CHECK(vec4_t(0.0, 0.0, 0.0, 1.0).isApprox(gmsckf.q_CI));

  MU_CHECK(gmsckf.enable_ns_trick);
  MU_CHECK(gmsckf.enable_qr_trick);

  return 0;
}

int test_GMSCKF_configure() {
  GMSCKF gmsckf;

  int retval = gmsckf.configure(TEST_CONFIG);

  // std::cout << gmsckf.P_imu << std::endl;
  // std::cout << gmsckf.Q_imu << std::endl;

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(15, gmsckf.P_imu.rows());
  MU_CHECK_EQ(15, gmsckf.P_imu.cols());
  MU_CHECK_EQ(6, gmsckf.P_cam.rows());
  MU_CHECK_EQ(6, gmsckf.P_cam.cols());
  MU_CHECK_EQ(15, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(6, gmsckf.P_imu_cam.cols());

  MU_CHECK_EQ(0, gmsckf.counter_frame_id);
  MU_CHECK(zeros(3, 1).isApprox(gmsckf.p_IC));
  MU_CHECK(vec4_t(0.5, -0.5, 0.5, -0.5).isApprox(gmsckf.q_CI));

  MU_CHECK(gmsckf.enable_ns_trick);
  MU_CHECK(gmsckf.enable_qr_trick);

  return 0;
}

int test_GMSCKF_P() {
  // Setup
  GMSCKF gmsckf;

  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());

  gmsckf.P_imu.fill(1.0);
  gmsckf.P_cam.fill(2.0);
  gmsckf.P_imu_cam.fill(3.0);

  // Test
  const matx_t P = gmsckf.P();

  // Assert
  const int imu_sz = 15;
  const int cam_sz = CameraState::size * gmsckf.N();

  matx_t P_imu_expected = zeros(imu_sz);
  P_imu_expected.fill(1.0);

  matx_t P_cam_expected = zeros(cam_sz);
  P_cam_expected.fill(2.0);

  matx_t P_imu_cam_expected = zeros(imu_sz, cam_sz);
  P_imu_cam_expected.fill(3.0);

  MU_CHECK_EQ(cam_sz, gmsckf.P_cam.rows());
  MU_CHECK_EQ(cam_sz, gmsckf.P_cam.cols());
  MU_CHECK(P.block(0, 0, imu_sz, imu_sz).isApprox(P_imu_expected));

  MU_CHECK_EQ(imu_sz, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(cam_sz, gmsckf.P_imu_cam.cols());
  MU_CHECK(P.block(0, imu_sz, imu_sz, cam_sz).isApprox(P_imu_cam_expected));

  MU_CHECK_EQ(imu_sz + cam_sz, P.cols());
  MU_CHECK_EQ(imu_sz + cam_sz, P.rows());
  MU_CHECK(P.block(imu_sz, imu_sz, cam_sz, cam_sz).isApprox(P_cam_expected));

  bool debug = false;
  if (debug) {
    mat2csv("/tmp/P.dat", P);
    PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/P.dat");
  }

  return 0;
}

int test_GMSCKF_J() {
  GMSCKF gmsckf;

  const vec4_t cam_q_CI = vec4_t{0.5, -0.5, 0.5, -0.5};
  const vec3_t cam_p_IC = vec3_t{0.0, 0.0, 0.0};
  const vec4_t q_hat_IG = vec4_t{0.0, 0.0, 0.0, 1.0};
  const int N = 1;

  const matx_t J = gmsckf.J(cam_q_CI, cam_p_IC, q_hat_IG, N);
  /* std::cout << "J: " << J.rows() << "x" << J.cols() << std::endl; */
  /* std::cout << J << std::endl; */

  MU_CHECK(J.rows() == 6);
  MU_CHECK(J.cols() == 21);

  return 0;
}

int test_GMSCKF_N() {
  GMSCKF gmsckf;
  MU_CHECK_EQ(0, gmsckf.N());

  gmsckf.augmentState(vec2_t::Zero());
  MU_CHECK_EQ(1, gmsckf.N());

  return 0;
}

int test_GMSCKF_H() {
  GMSCKF gmsckf;

  // Setup feature track
  const TrackID track_id = 0;
  const FrameID frame_id = 3;
  const auto data0 = Feature(vec2_t{0.0, 0.0});
  const auto data1 = Feature(vec2_t{0.0, 0.0});
  auto track = FeatureTrack(track_id, frame_id, data0, data1);
  track.type = DYNAMIC_STEREO_TRACK;

  // Setup track cam states
  gmsckf.p_IC = vec3_t{0.0, 0.0, 0.0};
  gmsckf.q_CI = vec4_t{0.5, -0.5, 0.5, -0.5};
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.p_G = vec3_t{0.1, 0.0, 0.0};
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.p_G = vec3_t{0.2, 0.0, 0.0};
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.p_G = vec3_t{0.3, 0.0, 0.0};
  gmsckf.augmentState(vec2_t::Zero());
  CameraStates track_cam_states = gmsckf.getTrackCameraStates(track);

  // Test
  const vec3_t p_G_f{1.0, 2.0, 3.0};
  matx_t H_f_j;
  matx_t H_x_j;
  gmsckf.H(track, track_cam_states, p_G_f, H_f_j, H_x_j);

  // Assert
  const int x_imu_sz = 15;
  const int x_cam_sz = 6;
  MU_CHECK_EQ(H_f_j.rows(), 8);
  MU_CHECK_EQ(H_f_j.cols(), 3);
  MU_CHECK_EQ(H_x_j.rows(), 8);
  MU_CHECK_EQ(H_x_j.cols(), x_imu_sz + gmsckf.N() * x_cam_sz);

  /* mat2csv("/tmp/H_f_j.dat", H_f_j); */
  /* mat2csv("/tmp/H_x_j.dat", H_x_j); */
  /* PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/H_f_j.dat"); */
  /* PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/H_x_j.dat"); */

  return 0;
}

int test_GMSCKF_initialize() {
  GMSCKF gmsckf;

  gmsckf.initialize(1e9);
  MU_CHECK_EQ(1e9, gmsckf.last_updated);

  return 0;
}

int test_GMSCKF_augmentState() {
  GMSCKF gmsckf;
  bool debug = false;

  // Augment state 1
  gmsckf.augmentState(vec2_t::Zero());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.rows());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.cols());
  MU_CHECK_EQ(gmsckf.x_cam_sz, gmsckf.P_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz, gmsckf.P_cam.cols());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz, gmsckf.P_imu_cam.cols());
  MU_CHECK_EQ(1, gmsckf.N());
  MU_CHECK_EQ(1, gmsckf.counter_frame_id);

  if (debug) {
    mat2csv("/tmp/P.dat", gmsckf.P());
    PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/P.dat");
  }

  // Augment state 2
  gmsckf.augmentState(vec2_t::Zero());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.rows());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.cols());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_cam.cols());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_imu_cam.cols());
  MU_CHECK_EQ(2, gmsckf.N());
  MU_CHECK_EQ(2, gmsckf.counter_frame_id);

  if (debug) {
    mat2csv("/tmp/P.dat", gmsckf.P());
    PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/P.dat");
  }

  // Augment state 3
  gmsckf.augmentState(vec2_t::Zero());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.rows());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu.cols());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_cam.cols());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(gmsckf.x_cam_sz * gmsckf.N(), gmsckf.P_imu_cam.cols());
  MU_CHECK_EQ(3, gmsckf.N());
  MU_CHECK_EQ(3, gmsckf.counter_frame_id);

  if (debug) {
    mat2csv("/tmp/P.dat", gmsckf.P());
    PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/P.dat");
  }

  return 0;
}

int test_GMSCKF_getTrackCameraStates() {
  GMSCKF gmsckf;
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());

  Feature f1{vec2_t{0.0, 0.0}};
  Feature f2{vec2_t{1.0, 1.0}};
  FeatureTrack track{0, 1, f1, f2};

  CameraStates track_cam_states = gmsckf.getTrackCameraStates(track);

  MU_CHECK_EQ(2, gmsckf.cam_states.size());
  MU_CHECK_EQ(2, track_cam_states.size());
  MU_CHECK_EQ(0, track_cam_states[0].frame_id);
  MU_CHECK_EQ(1, track_cam_states[1].frame_id);

  return 0;
}

int test_GMSCKF_predictionUpdate() {
  // Load raw dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0005", "sync");
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!",
              KITTI_RAW_DATASET.c_str());
    return -1;
  }

  // Prep blackbox
  BlackBox blackbox;
  if (blackbox.configure("/tmp", "test_msckf_predictionUpdate") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
  }

  // Setup MSCKF
  GMSCKF gmsckf;
  gmsckf.initialize(raw_dataset.oxts.timestamps[0],
                    euler2quat(raw_dataset.oxts.rpy[0]),
                    raw_dataset.oxts.v_G[0],
                    vec3_t{0.0, 0.0, 0.0});

  // Record initial conditions
  blackbox.recordTimeStep(raw_dataset.oxts.time[0],
                          raw_dataset.oxts.a_B[0],
                          raw_dataset.oxts.w_B[0],
                          raw_dataset.oxts.p_G[0],
                          raw_dataset.oxts.v_G[0],
                          raw_dataset.oxts.rpy[0],
                          raw_dataset.oxts.p_G[0],
                          raw_dataset.oxts.v_G[0],
                          raw_dataset.oxts.rpy[0]);

  // Loop through data and do prediction update
  for (int i = 1; i < (int) raw_dataset.oxts.time.size() - 1; i++) {
    const vec3_t a_B = raw_dataset.oxts.a_B[i];
    const vec3_t w_B = raw_dataset.oxts.w_B[i];
    const long ts = raw_dataset.oxts.timestamps[i];

    gmsckf.predictionUpdate(a_B, w_B, ts);
    blackbox.recordTimeStep(raw_dataset.oxts.time[i],
                            a_B,
                            w_B,
                            gmsckf.p_G,
                            gmsckf.v_G,
                            quat2euler(gmsckf.q_IG),
                            raw_dataset.oxts.p_G[i],
                            raw_dataset.oxts.v_G[i],
                            raw_dataset.oxts.rpy[i]);
  }

  bool debug = false;
  if (debug) {
    PYTHON_SCRIPT("scripts/plot_msckf.py /tmp/test_msckf_predictionUpdate");
  }

  return 0;
}

int test_GMSCKF_residualizeTrack() {
  // Camera model
  const int image_width = 640;
  const int image_height = 480;
  const double fov = 60.0;
  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  CameraProperty camera_property{0, fx, fy, cx, cy, image_width, image_height};

  // Setup MSCKF
  GMSCKF gmsckf;
  gmsckf.gimbal_model.tau_s << -0.045, -0.085, 0.080, 0.0, 0.0, 0.0;
  gmsckf.gimbal_model.tau_d << 0.01, 0.0, -0.03, 1.5707, 0.0, -1.5707;
  gmsckf.gimbal_model.w1 << 0.045, 0.0, 1.5707;
  gmsckf.gimbal_model.w2 << 0.0, 0.0, 0.0;
  gmsckf.gimbal_model.theta1_offset = -1.5707;
  gmsckf.gimbal_model.theta2_offset = 0.0;
  // -- Add first camera state
  gmsckf.initialize(0);
  // -- Add second camera state
  gmsckf.p_G = vec3_t{1.0, 1.0, 0.0};
  gmsckf.augmentState(vec2_t::Zero());

  // Prepare features and feature track
  // -- Create 2 features
  const vec3_t p_G_f{0.0, 0.0, 10.0};
  const mat3_t K = camera_property.K();
  // ---- Camera 0
  const mat3_t C_C0k_G = C(gmsckf.cam_states[0].q_CG);
  const mat3_t C_C0kp1_G = C(gmsckf.cam_states[1].q_CG);
  const vec3_t p_G_C0k = gmsckf.cam_states[0].p_G;
  const vec3_t p_G_C0kp1 = gmsckf.cam_states[1].p_G;
  vec2_t cam0_pt0 = pinhole_project(K, C_C0k_G, p_G_C0k, p_G_f);
  vec2_t cam0_pt1 = pinhole_project(K, C_C0kp1_G, p_G_C0kp1, p_G_f);
  cam0_pt0 = pinhole_pixel2ideal(camera_property.K(), cam0_pt0);
  cam0_pt1 = pinhole_pixel2ideal(camera_property.K(), cam0_pt1);
  Feature cam0_f0{cam0_pt0};
  Feature cam0_f1{cam0_pt1};
  // ---- Camera 1
  const mat4_t T_C1_C0 = gmsckf.gimbal_model.T_ds();
  const mat3_t C_C1_C0 = T_C1_C0.block(0, 0, 3, 3);
  const mat3_t C_C1k_G = C_C1_C0 * C_C0k_G;
  const mat3_t C_C1kp1_G = C_C1_C0 * C_C0kp1_G;
  const vec3_t p_C1_C0 = T_C1_C0.inverse().block(0, 3, 3, 1);
  const vec3_t p_G_C1k = p_C1_C0 + p_G_C0k;
  const vec3_t p_G_C1kp1 = p_C1_C0 + p_G_C0kp1;
  vec2_t cam1_pt0 = pinhole_project(K, C_C1k_G, p_G_C1k, p_G_f);
  vec2_t cam1_pt1 = pinhole_project(K, C_C1kp1_G, p_G_C1kp1, p_G_f);
  cam1_pt0 = pinhole_pixel2ideal(camera_property.K(), cam1_pt0);
  cam1_pt1 = pinhole_pixel2ideal(camera_property.K(), cam1_pt1);
  Feature cam1_f0{cam1_pt0};
  Feature cam1_f1{cam1_pt1};
  // -- Create a feature track based on two features
  FeatureTrack track;
  track.frame_start = 0;
  track.frame_end = 1;
  track.track0 = {cam0_f0, cam0_f1};
  track.track1 = {cam1_f0, cam1_f1};
  track.type = DYNAMIC_STEREO_TRACK;
  track.joint_angles.push_back(vec2_t::Zero());

  // Calculate track residual
  matx_t H_j;
  vecx_t r_j;
  int retval = gmsckf.residualizeTrack(track, H_j, r_j);

  // Assert
  MU_CHECK_EQ(0, retval);
  /* MU_CHECK_EQ(1, H_j.rows()); */
  /* MU_CHECK_EQ(27, H_j.cols()); */
  /* MU_CHECK_EQ(1, r_j.rows()); */
  /* MU_CHECK_EQ(1, r_j.cols()); */

  // MU_CHECK_EQ(0, retval);
  // for (int i = 0; i < r_j.rows(); i++) {
  //   MU_CHECK_NEAR(r_j(i), 0.0, 1e-5);
  // }

  // mat2csv("/tmp/H_j.dat", H_j);
  // mat2csv("/tmp/r_j.dat", r_j);
  // PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/H_j.dat");
  // PYTHON_SCRIPT("scripts/plot_matrix.py /tmp/r_j.dat");

  return 0;
}

int test_GMSCKF_calcResiduals() {
  // Camera model
  const int image_width = 640;
  const int image_height = 480;
  const double fov = 60.0;
  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  const mat3_t K = pinhole_K({fx, fy, cx, cy});
  PinholeModel pinhole_model{image_width, image_height, fx, fy, cx, cy};

  // Setup MSCKF
  GMSCKF gmsckf;
  // -- Modify default settings for test
  // -- Add first camera state
  gmsckf.initialize(0);
  // -- Add second camera state
  gmsckf.p_G = vec3_t{1.0, 1.0, 0.0};
  gmsckf.augmentState(vec2_t::Zero());

  // Prepare features and feature track
  // -- Create a feature track1
  const vec3_t p_G_f0{0.0, 0.0, 10.0};
  vec2_t pt0 = pinhole_model.project(p_G_f0,
                                     C(gmsckf.cam_states[0].q_CG),
                                     gmsckf.cam_states[0].p_G);
  vec2_t pt1 = pinhole_model.project(p_G_f0,
                                     C(gmsckf.cam_states[1].q_CG),
                                     gmsckf.cam_states[1].p_G);
  pt0 = pinhole_pixel2ideal(K, pt0);
  pt1 = pinhole_pixel2ideal(K, pt1);
  Feature f0{pt0};
  Feature f1{pt1};
  FeatureTrack track1{0, 1, f0, f1};
  // -- Create a feature track2
  const vec3_t p_G_f1{1.0, 1.0, 10.0};
  vec2_t pt2 = pinhole_model.project(p_G_f1,
                                     C(gmsckf.cam_states[0].q_CG),
                                     gmsckf.cam_states[0].p_G);
  vec2_t pt3 = pinhole_model.project(p_G_f1,
                                     C(gmsckf.cam_states[1].q_CG),
                                     gmsckf.cam_states[1].p_G);
  pt2 = pinhole_pixel2ideal(K, pt0);
  pt3 = pinhole_pixel2ideal(K, pt1);
  Feature f2{pt2};
  Feature f3{pt3};
  FeatureTrack track2{1, 1, f2, f3};
  // -- Create feature tracks
  FeatureTracks tracks{track1, track2};
  // FeatureTracks tracks{track1};

  // Calculate residuals
  matx_t T_H;
  vecx_t r_n;
  int retval = gmsckf.calcResiduals(tracks, T_H, r_n);
  print_shape("T_H", T_H);
  print_shape("r_n", r_n);

  // Assert
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_GMSCKF_correctIMUState() {
  // Setup MSCKF
  GMSCKF gmsckf;
  gmsckf.initialize(0);

  // Form correction vector
  const vec3_t dtheta_IG{0.0, 0.0, 0.0};
  const vec3_t db_g{1.0, 1.0, 1.0};
  const vec3_t dv_G{2.0, 2.0, 2.0};
  const vec3_t db_a{3.0, 3.0, 3.0};
  const vec3_t dp_G{4.0, 4.0, 4.0};

  vecx_t dx;
  dx.resize(15, 1);
  dx << dtheta_IG, db_g, dv_G, db_a, dp_G;

  // Correct IMU state
  gmsckf.correctIMUState(dx);

  // Assert
  MU_CHECK(gmsckf.v_G.isApprox(dv_G));
  MU_CHECK(gmsckf.p_G.isApprox(dp_G));

  return 0;
}

int test_GMSCKF_correctCameraStates() {
  // Setup MSCKF
  MSCKF msckf;
  msckf.initialize(0);

  // Form correction vector
  const vec3_t dtheta_IG{0.0, 0.0, 0.0};
  const vec3_t db_g{0.0, 0.0, 0.0};
  const vec3_t dv_G{0.0, 0.0, 0.0};
  const vec3_t db_a{0.0, 0.0, 0.0};
  const vec3_t dp_G_I{0.0, 0.0, 0.0};

  const vec3_t dtheta_CG{0.0, 0.0, 0.0};
  const vec3_t dp_G_C{1.0, 2.0, 3.0};

  vecx_t dx;
  dx.resize(21, 1);
  dx << dtheta_IG, db_g, dv_G, db_a, dp_G_I, dtheta_CG, dp_G_C;

  // Correct camera states
  msckf.correctCameraStates(dx);

  // Assert
  MU_CHECK(msckf.cam_states[0].p_G.isApprox(dp_G_C));

  return 0;
}

int test_GMSCKF_pruneCameraStates() {
  GMSCKF gmsckf;

  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());
  gmsckf.augmentState(vec2_t::Zero());

  gmsckf.max_window_size = 2;
  gmsckf.pruneCameraState();

  MU_CHECK_EQ(2, gmsckf.cam_states.size());
  MU_CHECK_EQ(2, gmsckf.cam_states[0].frame_id);
  MU_CHECK_EQ(3, gmsckf.cam_states[1].frame_id);
  MU_CHECK_EQ(CameraState::size * 2, gmsckf.P_cam.rows());
  MU_CHECK_EQ(CameraState::size * 2, gmsckf.P_cam.cols());
  MU_CHECK_EQ(gmsckf.x_imu_sz, gmsckf.P_imu_cam.rows());
  MU_CHECK_EQ(gmsckf.N() * gmsckf.x_cam_sz, gmsckf.P_imu_cam.cols());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GMSCKF_constructor);
  MU_ADD_TEST(test_GMSCKF_configure);
  MU_ADD_TEST(test_GMSCKF_initialize);
  MU_ADD_TEST(test_GMSCKF_P);
  MU_ADD_TEST(test_GMSCKF_J);
  MU_ADD_TEST(test_GMSCKF_N);
  MU_ADD_TEST(test_GMSCKF_H);
  MU_ADD_TEST(test_GMSCKF_initialize);
  MU_ADD_TEST(test_GMSCKF_augmentState);
  MU_ADD_TEST(test_GMSCKF_getTrackCameraStates);
  MU_ADD_TEST(test_GMSCKF_predictionUpdate);
  MU_ADD_TEST(test_GMSCKF_residualizeTrack);
  // MU_ADD_TEST(test_GMSCKF_calcResiduals);
  MU_ADD_TEST(test_GMSCKF_correctIMUState);
  MU_ADD_TEST(test_GMSCKF_correctCameraStates);
  MU_ADD_TEST(test_GMSCKF_pruneCameraStates);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
