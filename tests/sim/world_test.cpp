#include "msckf/imu_state.hpp"
#include "prototype/munit.hpp"
#include "sim/world.hpp"

namespace prototype {

#define TEST_MONO_CONFIG "test_configs/sim/sim_mono_camera.yaml"
#define TEST_STEREO_STATIC_CONFIG                                              \
  "test_configs/sim/sim_static_stereo_camera.yaml"
#define TEST_STEREO_DYNAMIC_CONFIG                                             \
  "test_configs/sim/sim_dynamic_stereo_camera.yaml"

int test_SimWorld_constructor() {
  SimWorld world;

  MU_CHECK_FLOAT(0.0, world.t);
  MU_CHECK_FLOAT(0.0, world.t_end);
  MU_CHECK_FLOAT(0.01, world.dt);

  return 0;
}

int test_SimWorld_configure() {
  SimWorld world;

  world.configure(TEST_MONO_CONFIG);
  MU_CHECK_FLOAT(0.0, world.t);
  MU_CHECK_FLOAT(0.01, world.dt);

  MU_CHECK_EQ(640, world.mono_camera.camera_model.image_width);
  MU_CHECK_EQ(640, world.mono_camera.camera_model.image_height);
  MU_CHECK_EQ(4, world.camera_motion.pos_points.size());
  MU_CHECK_EQ(4, world.camera_motion.att_points.size());

  return 0;
}

int test_SimWorld_create3DFeatures() {
  struct feature_bounds bounds;
  bounds.x_min = 1.0;
  bounds.x_max = 2.0;
  bounds.y_min = 3.0;
  bounds.y_max = 4.0;
  bounds.z_min = 5.0;
  bounds.z_max = 6.0;

  SimWorld world;
  const matx_t features = world.create3DFeatures(bounds, 1000);

  for (int i = 0; i < features.rows(); i++) {
    const vec3_t feature = features.block(i, 0, 1, 3).transpose();
    MU_CHECK(feature(0) >= bounds.x_min);
    MU_CHECK(feature(0) <= bounds.x_max);
    MU_CHECK(feature(1) >= bounds.y_min);
    MU_CHECK(feature(1) <= bounds.y_max);
    MU_CHECK(feature(2) >= bounds.z_min);
    MU_CHECK(feature(2) <= bounds.z_max);
  }

  return 0;
}

int test_SimWorld_create3DFeaturePerimeter() {
  // Test SimWorld.create3DFeaturePerimeter()
  SimWorld world;
  vec3_t origin{0.0, 0.0, 0.0};
  vec3_t dimensions{10.0, 10.0, 10.0};
  const matx_t features =
      world.create3DFeaturePerimeter(origin, dimensions, 1000);

  // Assert
  for (int i = 0; i < features.rows(); i++) {
    const vec3_t feature = features.block(i, 0, 1, 3).transpose();
    MU_CHECK(feature(0) >= origin(0) - dimensions(0) / 2.0);
    MU_CHECK(feature(0) <= origin(0) + dimensions(0) / 2.0);
    MU_CHECK(feature(1) >= origin(1) - dimensions(1) / 2.0);
    MU_CHECK(feature(1) <= origin(1) + dimensions(1) / 2.0);
    MU_CHECK(feature(2) >= origin(2) - dimensions(2) / 2.0);
    MU_CHECK(feature(2) <= origin(2) + dimensions(2) / 2.0);
  }

  // mat2csv("/tmp/features.dat", features);
  // PYTHON_SCRIPT("scripts/plot_world.py /tmp/features.dat");

  return 0;
}

int test_SimWorld_detectFeatures() {
  SimWorld world;

  // Test SimWorld.detectFeatures()
  world.configure(TEST_MONO_CONFIG);
  world.detectFeatures();
  for (int i = 0; i < 100; i++) {
    world.step();
  }
  world.detectFeatures();

  // Assert
  MU_CHECK(world.features_tracking.size() > 0);
  MU_CHECK(world.tracks_tracking.size() > 0);
  MU_CHECK(world.tracks_lost.size() > 0);

  return 0;
}

int test_SimWorld_getLostTracks() {
  SimWorld world;

  // Test SimWorld.getLostTracks()
  world.configure(TEST_MONO_CONFIG);
  world.detectFeatures();
  for (int i = 0; i < 1000; i++) {
    // std::cout << "frame: " << i << std::endl;
    world.step();
  }
  FeatureTracks tracks = world.getLostTracks();

  // Assert
  MU_CHECK(tracks.size() > 0);
  MU_CHECK_EQ(0, world.tracks_lost.size());

  return 0;
}

int test_SimWorld_generateInitializationData() {
  SimWorld world;

  const int nb_samples = 10;
  std::vector<vec3_t> imu_gyro_buffer;
  std::vector<vec3_t> imu_accel_buffer;
  world.generateInitializationData(nb_samples,
                                   imu_gyro_buffer,
                                   imu_accel_buffer);

  for (int i = 0; i < nb_samples; i++) {
    std::cout << imu_gyro_buffer[i].transpose() << std::endl;
    std::cout << imu_accel_buffer[i].transpose() << std::endl;
  }

  return 0;
}

int test_SimWorld_step_mono_camera() {
  SimWorld world;

  // Test step
  world.configure(TEST_MONO_CONFIG);
  while (world.t <= 10.0) {
    world.step();
  }
  FeatureTracks tracks = world.getLostTracks();

  // Assert
  MU_CHECK(tracks.size() > 0);
  MU_CHECK_EQ(0, world.tracks_lost.size());

  // Plot
  // PYTHON_SCRIPT("scripts/plot_sim.py /tmp/sim");

  return 0;
}

int test_SimWorld_step_stereo_static_camera() {
  SimWorld world;
  IMUState imu;

  // Test step
  if (world.configure(TEST_STEREO_STATIC_CONFIG) != 0) {
    FATAL("Failed to configure simulation!");
  }
  imu.q_IG = euler2quat(world.camera_motion.rpy_G);
  imu.p_G = world.camera_motion.p_G;
  imu.v_G = world.camera_motion.v_G;

  while (world.step() == 0) {
    const vec3_t a_m = world.camera_motion.a_B;
    const vec3_t w_m = world.camera_motion.w_B;
    imu.update(a_m, w_m, world.dt);
  }
  FeatureTracks tracks = world.getLostTracks();
  for (const auto &track : tracks) {
    MU_CHECK(track.T_cam1_cam0.isApprox(mat4_t::Zero()) == false);
  }

  // Assert
  MU_CHECK(tracks.size() > 0);
  MU_CHECK_EQ(0, world.tracks_lost.size());

  std::cout << world.camera_motion.p_G.transpose() << std::endl;
  std::cout << imu.p_G.transpose() << std::endl;

  // Plot
  // PYTHON_SCRIPT("scripts/plot_sim.py /tmp/sim");

  return 0;
}

int test_SimWorld_step_stereo_dynamic_camera() {
  SimWorld world;
  IMUState imu;

  // Test step
  if (world.configure(TEST_STEREO_DYNAMIC_CONFIG) != 0) {
    FATAL("Failed to configure simulation!");
  }
  imu.q_IG = euler2quat(world.camera_motion.rpy_G);
  imu.p_G = world.camera_motion.p_G;
  imu.v_G = world.camera_motion.v_G;

  while (world.step() == 0) {
    const vec3_t a_m = world.camera_motion.a_B;
    const vec3_t w_m = world.camera_motion.w_B;
    imu.update(a_m, w_m, world.dt);
  }
  FeatureTracks tracks = world.getLostTracks();

  // Assert
  MU_CHECK(tracks.size() > 0);
  MU_CHECK_EQ(0, world.tracks_lost.size());

  std::cout << world.camera_motion.p_G.transpose() << std::endl;
  std::cout << imu.p_G.transpose() << std::endl;

  // Plot
  // PYTHON_SCRIPT("scripts/plot_sim.py /tmp/sim");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_SimWorld_constructor);
  MU_ADD_TEST(test_SimWorld_configure);
  MU_ADD_TEST(test_SimWorld_create3DFeatures);
  MU_ADD_TEST(test_SimWorld_create3DFeaturePerimeter);
  MU_ADD_TEST(test_SimWorld_detectFeatures);
  MU_ADD_TEST(test_SimWorld_getLostTracks);
  MU_ADD_TEST(test_SimWorld_generateInitializationData);
  MU_ADD_TEST(test_SimWorld_step_mono_camera);
  MU_ADD_TEST(test_SimWorld_step_stereo_static_camera);
  MU_ADD_TEST(test_SimWorld_step_stereo_dynamic_camera);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
