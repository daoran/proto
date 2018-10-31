#include "camera/pinhole_model.hpp"
#include "dataset/kitti/kitti.hpp"
#include "feature2d/stereo_klt_tracker.hpp"
#include "msckf/feature_estimator.hpp"
#include "prototype/munit.hpp"
#include "sim/sim.hpp"

namespace prototype {

static const std::string DATASET_PATH = "/data/kitti/raw";
static const std::string SIM_MONO_CONFIG_PATH =
    "test_configs/msckf/feature_estimator_mono_sim.yaml";
static const std::string SIM_STEREO_CONFIG_PATH =
    "test_configs/msckf/feature_estimator_stereo_sim.yaml";

struct mono_test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  const vec3_t landmark{0.0, 0.0, 10.0};

  mono_test_config() {}
};

struct stereo_test_config {
  RawDataset raw_dataset;
  StereoKLTTracker tracker;

  stereo_test_config() {}
};

void setup_mono_test(const struct mono_test_config &config,
                     CameraStates &track_cam_states,
                     FeatureTrack &track) {
  // Camera model
  PinholeModel cam_model;
  cam_model = PinholeModel{config.image_width,
                           config.image_height,
                           config.fx,
                           config.fy,
                           config.cx,
                           config.cy};

  // -- Camera state 0
  const vec3_t p_G_C0{0.0, 0.0, 0.0};
  const vec3_t rpy_C0G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const vec4_t q_C0G = euler2quat(rpy_C0G);
  const mat3_t C_C0G = C(q_C0G);
  const CameraState cam_state0{p_G_C0, q_C0G};
  // -- Camera state 1
  const vec3_t p_G_C1{1.0, 1.0, 0.0};
  const vec3_t rpy_C1G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const vec4_t q_C1G = euler2quat(rpy_C1G);
  const mat3_t C_C1G = C(q_C1G);
  const CameraState cam_state1{p_G_C1, q_C1G};
  // -- Add to track camera states
  track_cam_states.push_back(cam_state0);
  track_cam_states.push_back(cam_state1);

  // Feature track
  // -- Project landmark to pixel coordinates
  const vec3_t landmark{config.landmark};
  const vec2_t kp1 = cam_model.project(landmark, C_C0G, p_G_C0);
  const vec2_t kp2 = cam_model.project(landmark, C_C1G, p_G_C1);
  // -- Convert pixel coordinates to image coordinates
  const vec2_t pt1 = cam_model.pixel2ideal(kp1);
  const vec2_t pt2 = cam_model.pixel2ideal(kp2);
  // -- Add to feature track
  track = FeatureTrack{0, 1, Feature{pt1}, Feature{pt2}};
}

struct stereo_test_config setup_stereo_test() {
  struct stereo_test_config config;

  // Load RAW KITTI dataset
  config.raw_dataset =
      RawDataset(DATASET_PATH, "2011_09_26", "0001", "extract");
  if (config.raw_dataset.load() != 0) {
    exit(-1);
  }

  // Obtain cam0 to cam1 transform
  const mat3_t R_cam1_cam0 = config.raw_dataset.calib_cam_to_cam.R[1];
  const vec3_t t_cam1_cam0 = config.raw_dataset.calib_cam_to_cam.T[1];
  const mat4_t T_cam1_cam0 = transformation_matrix(R_cam1_cam0, t_cam1_cam0);

  // Create camera properties
  const vec2_t image_size = config.raw_dataset.calib_cam_to_cam.S[0];
  const mat3_t cam0_K = config.raw_dataset.calib_cam_to_cam.K[0];
  const vecx_t cam0_D = config.raw_dataset.calib_cam_to_cam.D[0];
  const mat3_t cam1_K = config.raw_dataset.calib_cam_to_cam.K[1];
  const vecx_t cam1_D = config.raw_dataset.calib_cam_to_cam.D[1];
  CameraProperty cam0(0, "pinhole", cam0_K, "radtan", cam0_D, image_size);
  CameraProperty cam1(1, "pinhole", cam1_K, "radtan", cam1_D, image_size);

  // Initialize tracker
  config.tracker = StereoKLTTracker(cam0, cam1, T_cam1_cam0, 5, 20);

  return config;
}

int test_lls_triangulation() {
  // Camera model
  struct mono_test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);

  // Create keypoints
  const vec3_t landmark{1.0, 0.0, 10.0};
  const vec2_t z1 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.0});
  const vec2_t z2 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.2});

  const vec3_t u1 = z1.homogeneous();
  const vec3_t u2 = z2.homogeneous();

  const mat34_t P1{cam_model.P(I(3), vec3_t{0.0, 0.0, 0.0})};
  const mat34_t P2{cam_model.P(I(3), vec3_t{0.0, 0.0, 0.2})};

  std::cout << "u1: " << u1.transpose() << std::endl;
  std::cout << "u2: " << u2.transpose() << std::endl;
  std::cout << "P1: " << P1 << std::endl;
  std::cout << "P2: " << P2 << std::endl;

  const vec3_t X = lls_triangulation(u1, P1, u2, P2);
  std::cout << "X:" << std::endl;
  std::cout << X << std::endl;

  return 0;
}

int test_lls_triangulation2() {
  // Camera model
  struct mono_test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);

  // Create keypoints
  const vec3_t landmark{0.0, 0.0, 10.0};
  const vec2_t kp1 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.0});
  const vec2_t kp2 = cam_model.project(landmark, I(3), vec3_t{0.2, 0.0, 0.0});
  const vec2_t z1 = cam_model.pixel2ideal(kp1);
  const vec2_t z2 = cam_model.pixel2ideal(kp2);

  // Camera states
  // -- Camera state 0
  const vec3_t p_G_C0{0.0, 0.0, 0.0};
  const vec4_t q_C0G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C0G = C(q_C0G);
  // -- Camera state 1
  const vec3_t p_G_C1{0.0, -0.2, 0.0};
  const vec4_t q_C1G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C1G = C(q_C1G);

  // Calculate rotation and translation of first and last camera states
  // -- Obtain rotation and translation from camera 0 to camera 1
  const mat3_t C_C0C1 = C_C0G * C_C1G.transpose();
  const vec3_t t_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
  const mat4_t T_C0_C1 = transformation_matrix(C_C0C1, t_C0C1);
  const mat4_t T_C1_C0 = T_C0_C1.inverse();

  // Triangulate
  const vec3_t X = lls_triangulation(z1, z2, T_C1_C0);
  MU_CHECK(X.isApprox(landmark));

  return 0;
}

int test_triangulate_mono_track() {
  // Camera model
  struct mono_test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);
  // Camera states
  // -- Camera state 0
  const vec3_t p_G_C0{0.0, 0.0, 0.0};
  const vec4_t q_C0G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C0G = C(q_C0G);
  // -- Camera state 1
  const vec3_t p_G_C1{0.0, -0.2, -0.1};
  const vec4_t q_C1G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C1G = C(q_C1G);

  // Calculate rotation and translation of first and last camera states
  // -- Obtain rotation and translation from camera 0 to camera 1
  const mat3_t C_C0C1 = C_C0G * C_C1G.transpose();
  const vec3_t t_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
  const mat4_t T_C0_C1 = transformation_matrix(C_C0C1, t_C0C1);
  const mat4_t T_C1_C0 = T_C0_C1.inverse();

  // Create feature tracks
  FeatureTracks tracks;
  std::vector<vec3_t> landmarks;

  for (int i = 0; i < 1000; i++) {
    // Create keypoints
    const double x = randf(-10.0, 10.0);
    const double y = randf(-10.0, 10.0);
    const double z = randf(-10.0, 10.0);
    const vec3_t landmark{x, y, z};
    const vec2_t kp1 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.0});
    const vec2_t kp2 = cam_model.project(landmark, I(3), vec3_t{0.2, 0.1, 0.0});
    const vec2_t z1 = cam_model.pixel2ideal(kp1);
    const vec2_t z2 = cam_model.pixel2ideal(kp2);

    // Create Feature Track
    Feature f0{z1};
    Feature f1{z2};
    FeatureTrack track{0, 1, f0, f1};

    // Add to landmarks and tracks
    landmarks.push_back(landmark);
    tracks.push_back(track);
  }
  triangulate_mono_tracks(T_C1_C0, tracks);

  // Assert
  for (int i = 0; i < 1000; i++) {
    MU_CHECK((tracks[i].p_C0_f - landmarks[i]).norm() < 1e-2);
  }

  return 0;
}

int test_group_tracks() {
  // Setup
  FeatureTracks tracks;
  std::map<FrameID, FeatureTracks> expected_track_groups;
  std::set<FrameID> expected_frame_starts;
  mat4_t T_cam1_cam0 = I(4);

  for (int i = 0; i < 10; i++) {
    const TrackID track_id = i;
    const int track_length = randi(0, 10);
    const int frame_start = randi(0, 10);
    const int frame_end = frame_start + track_length - 1;

    const Features track0(track_length, Feature());
    const Features track1(track_length, Feature());
    const FeatureTrack track(track_id,
                             frame_start,
                             frame_end,
                             track0,
                             track1,
                             T_cam1_cam0);
    tracks.push_back(track);

    expected_track_groups[frame_start].push_back(track);
    expected_frame_starts.insert(frame_start);
  }

  // Test
  std::map<FrameID, FeatureTracks> track_groups;
  std::set<FrameID> frame_starts;
  group_tracks(tracks, track_groups, frame_starts);

  // Assert
  for (auto &group : track_groups) {
    FrameID frame_start = group.first;
    FeatureTracks tracks = group.second;

    std::cout << "frame_start: " << frame_start << std::endl;
    std::cout << "group size: " << tracks.size() << std::endl;
  }

  return 0;
}

int test_triangulate_stereo_track() {
  // Camera model
  struct mono_test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);
  // Camera states
  // -- Camera state 0
  const vec3_t p_G_C0{0.0, 0.0, 0.0};
  const vec4_t q_C0G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C0G = C(q_C0G);
  // -- Camera state 1
  const vec3_t p_G_C1{0.0, -0.2, -0.1};
  const vec4_t q_C1G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C1G = C(q_C1G);

  // Calculate rotation and translation of first and last camera states
  // -- Obtain rotation and translation from camera 0 to camera 1
  const mat3_t C_C0C1 = C_C0G * C_C1G.transpose();
  const vec3_t t_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
  const mat4_t T_C0_C1 = transformation_matrix(C_C0C1, t_C0C1);
  const mat4_t T_C1_C0 = T_C0_C1.inverse();

  // Create feature tracks
  FeatureTracks tracks;
  std::vector<vec3_t> landmarks;

  for (int i = 0; i < 1000; i++) {
    // Create keypoints
    const double x = randf(-10.0, 10.0);
    const double y = randf(-10.0, 10.0);
    const double z = randf(-10.0, 10.0);
    const vec3_t landmark{x, y, z};
    const vec2_t kp1 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.0});
    const vec2_t kp2 = cam_model.project(landmark, I(3), vec3_t{0.2, 0.1, 0.0});
    const vec2_t z1 = cam_model.pixel2ideal(kp1);
    const vec2_t z2 = cam_model.pixel2ideal(kp2);

    // Create Feature Track
    Feature f0{z1};
    Feature f1{z2};
    FeatureTrack track;
    track.track_id = 0;
    track.frame_start = 0;
    track.frame_end = 0;
    track.track0 = {f0};
    track.track1 = {f1};
    track.T_cam1_cam0 = {T_C1_C0};

    // Add to landmarks and tracks
    landmarks.push_back(landmark);
    tracks.push_back(track);
  }
  tracks = triangulate_stereo_tracks(T_C1_C0, tracks);

  // Assert
  for (int i = 0; i < 1000; i++) {
    // std::cout << landmarks[i].transpose() << std::endl;
    // std::cout << tracks[i].p_C0_f.transpose() << std::endl;
    MU_CHECK((tracks[i].p_C0_f - landmarks[i]).norm() < 1e-2);
  }

  return 0;
}

int test_FeatureEstimator_triangulate() {
  // Camera model
  struct mono_test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);

  // Camera states
  // -- Camera state 0
  const vec3_t p_G_C0{0.0, 0.0, 0.0};
  const vec4_t q_C0G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C0G = C(q_C0G);
  // -- Camera state 1
  const vec3_t p_G_C1{2.0, 0.0, 0.0};
  const vec4_t q_C1G = vec4_t{0.5, -0.5, 0.5, -0.5};
  const mat3_t C_C1G = C(q_C1G);

  // Features
  const vec3_t landmark{1.0, 0.0, 10.0};
  const vec2_t kp1 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 0.0});
  const vec2_t kp2 = cam_model.project(landmark, I(3), vec3_t{0.0, 0.0, 2.0});
  // -- Convert pixel coordinates to image coordinates
  const vec2_t pt1 = cam_model.pixel2ideal(kp1);
  const vec2_t pt2 = cam_model.pixel2ideal(kp2);
  // -- Add to feature track
  const Feature f1 = Feature{pt1};
  const Feature f2 = Feature{pt2};
  const FeatureTrack track{0, 1, f1, f2};

  // Calculate rotation and translation of first and last camera states
  // -- Obtain rotation and translation from camera 0 to camera 1
  const mat3_t C_C0C1 = C_C0G * C_C1G.transpose();
  const vec3_t t_C0_C1C0 = C_C0G * (p_G_C1 - p_G_C0);

  // Triangulate
  vec3_t p_C0_f;
  int retval =
      FeatureEstimator::triangulate(pt1, pt2, C_C0C1, t_C0_C1C0, p_C0_f);
  // std::cout << "p_C0_f: " << p_C0_f.transpose() << std::endl;

  // // Transform feature from camera 0 to global frame
  // vec3_t p_G_f = C_C0G.transpose() * p_C0_f + p_G_C0;
  // std::cout << "p_G_f: " << p_G_f.transpose() << std::endl;

  // Assert
  MU_CHECK(landmark.isApprox(p_C0_f));
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_FeatureEstimator_initialEstimate() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Initial estimate
  vec3_t p_C0_f;
  FeatureEstimator estimator(track, track_cam_states);
  int retval = estimator.initialEstimate(p_C0_f);

  // Assert
  MU_CHECK(((config.landmark - p_C0_f).norm() < 1e-6));
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_FeatureEstimator_jacobian() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Test jacobian
  vec3_t p_C0_f;
  vec3_t x{0.0, 0.0, 0.1};
  FeatureEstimator estimator(track, track_cam_states);
  // estimator.jacobian(x);

  return 0;
}

int test_FeatureEstimator_reprojectionError() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Test jacobian
  vec3_t p_C0_f;
  vec3_t x{0.0, 0.0, 0.1};
  FeatureEstimator estimator(track, track_cam_states);
  // estimator.reprojectionError(x);

  return 0;
}

int test_FeatureEstimator_estimate() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Test jacobian
  vec3_t p_G_f;
  FeatureEstimator estimator(track, track_cam_states);
  estimator.debug_mode = true;

  struct timespec start = tic();
  int retval = estimator.estimate(p_G_f);
  printf("elasped: %fs\n", toc(&start));
  std::cout << p_G_f.transpose() << std::endl;

  MU_CHECK_EQ(0, retval);
  MU_CHECK(((config.landmark - p_G_f).norm() < 1e-6));

  return 0;
}

int test_AnalyticalReprojectionError_constructor() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Get camera 0 rotation and translation
  const mat3_t C_C0G = C(track_cam_states[0].q_CG);
  const vec3_t p_G_C0 = track_cam_states[0].p_G;
  // Get camera i rotation and translation
  const int camera_index = 0;
  const mat3_t C_CiG = C(track_cam_states[camera_index].q_CG);
  const vec3_t p_G_Ci = track_cam_states[camera_index].p_G;
  // Set camera 0 as origin, work out rotation and translation
  // between camera i to to camera 0
  const mat3_t C_CiC0 = C_CiG * C_C0G.transpose();
  const vec3_t t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

  // Ceres reprojection error
  AnalyticalReprojectionError error{C_CiC0,
                                    t_Ci_CiC0,
                                    track.track[camera_index].getKeyPoint()};

  return 0;
}

int test_AnalyticalReprojectionError_evaluate() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Get camera 0 rotation and translation
  const mat3_t C_C0G = C(track_cam_states[0].q_CG);
  const vec3_t p_G_C0 = track_cam_states[0].p_G;

  for (int i = 0; i < 2; i++) {
    // Get camera 0 rotation and translation
    const mat3_t C_CiG = C(track_cam_states[i].q_CG);
    const vec3_t p_G_Ci = track_cam_states[i].p_G;
    // Set camera 0 as origin, work out rotation and translation
    // between camera 0 to to camera 0
    const mat3_t C_CiC0 = C_CiG * C_C0G.transpose();
    const vec3_t t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

    // Calculate reprojection error for first measurement
    double r[2] = {1.0, 1.0};
    AnalyticalReprojectionError error{C_CiC0,
                                      t_Ci_CiC0,
                                      track.track[i].getKeyPoint()};

    // Create inverse depth params (these are to be optimized)
    const double alpha = config.landmark(0) / config.landmark(2);
    const double beta = config.landmark(1) / config.landmark(2);
    const double rho = 1.0 / config.landmark(2);
    double *x = (double *) malloc(sizeof(double) * 3);
    x[0] = alpha;
    x[1] = beta;
    x[2] = rho;

    // Test and assert
    error.Evaluate(&x, r, NULL);
    std::cout << "residuals: " << r[0] << ", " << r[1] << std::endl;

    MU_CHECK_NEAR(0.0, r[0], 1e-5);
    MU_CHECK_NEAR(0.0, r[1], 1e-5);
  }

  return 0;
}

int test_CeresFeatureEstimator_constructor() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{track, track_cam_states};

  return 0;
}

int test_CeresFeatureEstimator_setupProblem() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{track, track_cam_states};
  estimator.setupProblem();

  return 0;
}

int test_CeresFeatureEstimator_estimate() {
  // Setup test
  const struct mono_test_config config;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_mono_test(config, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{track, track_cam_states};

  vec3_t p_G_f;
  struct timespec start = tic();
  estimator.estimate(p_G_f);
  printf("elasped: %fs\n", toc(&start));

  // std::cout << p_G_f.transpose() << std::endl;

  return 0;
}

int test_CeresFeatureEstimator_estimate_mono_sim() {
  // Setup simulation
  SimWorld sim;
  if (sim.configure(SIM_MONO_CONFIG_PATH) != 0) {
    LOG_ERROR("Failed to configure simulation!");
    return -1;
  }

  // IMU to camera extrinsics
  const vec3_t ext_p_IC{0.0, 0.0, 0.0};
  const vec4_t ext_q_CI{0.50243, -0.491157, 0.504585, -0.50172};

  // Simulate
  CameraStates camera_states;
  std::vector<vec3_t> estimates;
  std::vector<vec3_t> ground_truths;

  // Add first camera state
  const vec3_t imu_p_G = sim.camera_motion.p_G;
  const vec4_t imu_q_IG = euler2quat(sim.camera_motion.rpy_G);
  const vec4_t cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
  const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
  camera_states.emplace_back(0, cam_p_G, cam_q_CG);

  for (int i = 1; i < 100; i++) {
    sim.step();

    // Add camera state
    const vec3_t imu_p_G = sim.camera_motion.p_G;
    const vec4_t imu_q_IG = euler2quat(sim.camera_motion.rpy_G);
    const vec4_t cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
    const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
    camera_states.emplace_back(i, cam_p_G, cam_q_CG);

    // Get lost tracks
    FeatureTracks tracks = sim.getLostTracks();

    // Estimate features
    int added = 0;
    for (auto track : tracks) {
      auto track_cam_states = get_track_camera_states(camera_states, track);
      assert(track_cam_states.size() == track.trackedLength());
      CeresFeatureEstimator estimator{track, track_cam_states};

      vec3_t p_G_f;
      if (estimator.estimate(p_G_f) == 0) {
        estimates.emplace_back(p_G_f);
        ground_truths.emplace_back(track.track[0].ground_truth);
        added++;
      }
    }

    // Assert
    for (size_t i = 0; i < estimates.size(); i++) {
      auto est = estimates[i];
      auto gnd = ground_truths[i];
      auto diff = (est - gnd).norm();
      std::cout << "est: " << est.transpose() << std::endl;
      std::cout << "gnd: " << gnd.transpose() << std::endl;
      std::cout << "diff: " << diff << std::endl;
      // MU_CHECK(diff < 1.0);
    }

    // Show inliers vs outliers
    int total = tracks.size();
    LOG_INFO("Estimated [%d / %d]", added, total);
  }

  return 0;
}

int test_CeresFeatureEstimator_estimate_stereo_sim() {
  // Setup simulation
  SimWorld sim;
  if (sim.configure(SIM_STEREO_CONFIG_PATH) != 0) {
    LOG_ERROR("Failed to configure simulation!");
    return -1;
  }
  const mat4_t T_cam1_cam0 = sim.stereo_camera.T_cam1_cam0;

  // IMU to camera extrinsics
  const vec3_t ext_p_IC{0.0, 0.0, 0.0};
  const vec4_t ext_q_CI{0.50243, -0.491157, 0.504585, -0.50172};

  // Simulate
  CameraStates camera_states;
  std::vector<vec3_t> estimates;
  std::vector<vec3_t> ground_truths;

  // Add first camera state
  const vec3_t imu_p_G = sim.camera_motion.p_G;
  const vec4_t imu_q_IG = euler2quat(sim.camera_motion.rpy_G);
  const vec4_t cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
  const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
  camera_states.emplace_back(0, cam_p_G, cam_q_CG);

  for (int i = 1; i < 100; i++) {
    sim.step();

    // Add camera state
    const vec3_t imu_p_G = sim.camera_motion.p_G;
    const vec4_t imu_q_IG = euler2quat(sim.camera_motion.rpy_G);
    const vec4_t cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
    const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
    camera_states.emplace_back(i, cam_p_G, cam_q_CG);

    // Get lost tracks
    FeatureTracks tracks = sim.getLostTracks();
    if (tracks.size() > 0) {
      for (auto track : tracks) {
        MU_CHECK(track.type == STATIC_STEREO_TRACK);
      }
    }

    // Estimate features
    int added = 0;
    for (auto track : tracks) {
      auto track_cam_states = get_track_camera_states(camera_states, track);
      assert(track_cam_states.size() == track.trackedLength());
      CeresFeatureEstimator estimator{track, track_cam_states};

      vec3_t p_G_f;
      if (estimator.estimate(p_G_f) == 0) {
        estimates.emplace_back(p_G_f);
        ground_truths.emplace_back(track.track0[0].ground_truth);
        added++;
      }
    }

    // Assert
    for (size_t i = 0; i < estimates.size(); i++) {
      auto est = estimates[i];
      auto gnd = ground_truths[i];
      std::cout << "est: " << est.transpose() << std::endl;
      std::cout << "gnd: " << gnd.transpose() << std::endl;
      std::cout << "diff: " << (est - gnd).norm() << std::endl;
      // MU_CHECK(diff < 1.0);
    }

    // Show inliers vs outliers
    int total = tracks.size();
    LOG_INFO("Estimated [%d / %d]", added, total);
  }

  return 0;
}

int test_CeresFeatureEstimator_estimate_stereo_kitti() {
  struct stereo_test_config test = setup_stereo_test();
  test.tracker.show_matches = true;

  // IMU to camera extrinsics
  const vec3_t ext_p_IC{0.0, 0.0, 0.0};
  const vec4_t ext_q_CI{0.50243, -0.491157, 0.504585, -0.50172};

  CameraStates camera_states;
  std::vector<vec3_t> landmarks;
  for (int i = 0; i < 100; i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);

    // Add camera state
    const vec3_t imu_p_G = test.raw_dataset.oxts.p_G[i];
    const vec4_t imu_q_IG = euler2quat(test.raw_dataset.oxts.rpy[i]);
    const vec4_t cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
    const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
    camera_states.emplace_back(i, cam_p_G, cam_q_CG);

    // Track features and get lost tracks
    test.tracker.update(cam0_img, cam1_img);
    auto tracks = test.tracker.getLostTracks();

    // Triangulate tracks
    // triangulate_tracks(T_cam1_cam0, tracks);

    // Estimate features
    int landmarks_added = 0;
    for (auto track : tracks) {
      auto track_cam_states = get_track_camera_states(camera_states, track);
      assert(track_cam_states.size() == track.trackedLength());
      CeresFeatureEstimator estimator{track, track_cam_states};

      vec3_t p_G_f;
      if (estimator.estimate(p_G_f) == 0) {
        landmarks.emplace_back(p_G_f);
        landmarks_added++;
      }
    }

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  // Write list of landmarks to file
  std::ofstream landmarks_file("/tmp/landmarks.csv");
  for (auto landmark : landmarks) {
    landmarks_file << landmark(0) << ",";
    landmarks_file << landmark(1) << ",";
    landmarks_file << landmark(2) << std::endl;
  }
  landmarks_file.close();

  // Output ground truth to file
  LOG_INFO("Outputting ground truth to file!");
  std::ofstream pose_file("/tmp/pose.csv");
  pose_file << "x,y,z,roll,pitch,yaw" << std::endl;
  for (auto camera_state : camera_states) {
    // Position
    pose_file << camera_state.p_G(0) << ",";
    pose_file << camera_state.p_G(1) << ",";
    pose_file << camera_state.p_G(2) << ",";

    // Roll, pitch and yaw
    const vec3_t rpy_G = quat2euler(camera_state.q_CG);
    pose_file << rpy_G(0) << ",";
    pose_file << rpy_G(1) << ",";
    pose_file << rpy_G(2) << std::endl;
  }
  pose_file.close();

  return 0;
}

void test_suite() {
  // FeatureEstimator
  // MU_ADD_TEST(test_lls_triangulation);
  // MU_ADD_TEST(test_lls_triangulation2);
  // MU_ADD_TEST(test_triangulate_mono_track);
  // MU_ADD_TEST(test_group_tracks);
  MU_ADD_TEST(test_triangulate_stereo_track);
  // MU_ADD_TEST(test_FeatureEstimator_triangulate);
  // MU_ADD_TEST(test_FeatureEstimator_initialEstimate);
  // MU_ADD_TEST(test_FeatureEstimator_jacobian);
  // MU_ADD_TEST(test_FeatureEstimator_reprojectionError);
  // MU_ADD_TEST(test_FeatureEstimator_estimate);

  // AnalyticalReprojectionError
  // MU_ADD_TEST(test_AnalyticalReprojectionError_constructor);
  // MU_ADD_TEST(test_AnalyticalReprojectionError_evaluate);

  // CeresFeatureEstimator
  // MU_ADD_TEST(test_CeresFeatureEstimator_constructor);
  // MU_ADD_TEST(test_CeresFeatureEstimator_setupProblem);
  // MU_ADD_TEST(test_CeresFeatureEstimator_estimate);
  // MU_ADD_TEST(test_CeresFeatureEstimator_estimate_mono_sim);
  // MU_ADD_TEST(test_CeresFeatureEstimator_estimate_stereo_sim);
  // MU_ADD_TEST(test_CeresFeatureEstimator_estimate_stereo_kitti);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
