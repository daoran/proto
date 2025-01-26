#include "munit.h"
#include "xyz_sim.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"
#define TEST_SIM_GIMBAL TEST_DATA_PATH "sim_gimbal"


// SIM FEATURES //////////////////////////////////////////////////////////////

int test_sim_features_load(void) {
  const char *csv_file = TEST_SIM_DATA "/features.csv";
  sim_features_t *features_data = sim_features_load(csv_file);
  MU_ASSERT(features_data->num_features > 0);
  sim_features_free(features_data);
  return 0;
}

// SIM IMU DATA //////////////////////////////////////////////////////////////

int test_sim_imu_data_load(void) {
  // const char *csv_file = TEST_SIM_DATA "/imu0/data.csv";
  // sim_imu_data_t *imu_data = sim_imu_data_load(csv_file);
  // sim_imu_data_free(imu_data);
  return 0;
}

// SIM CAMERA DATA ///////////////////////////////////////////////////////////

int test_sim_camera_frame_load(void) {
  const char *frame_csv = TEST_SIM_DATA "/cam0/data/100000000.csv";
  sim_camera_frame_t *frame_data = sim_camera_frame_load(frame_csv);

  MU_ASSERT(frame_data != NULL);
  MU_ASSERT(frame_data->ts == 100000000);
  MU_ASSERT(frame_data->feature_ids[0] == 1);

  sim_camera_frame_free(frame_data);

  return 0;
}

int test_sim_camera_data_load(void) {
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);
  sim_camera_data_free(cam_data);
  return 0;
}

int test_sim_camera_circle_trajectory(void) {
  // Simulate features
  const real_t origin[3] = {0.0, 0.0, 0.0};
  const real_t dim[3] = {5.0, 5.0, 5.0};
  const int num_features = 1000;
  real_t features[3 * 1000] = {0};
  sim_create_features(origin, dim, num_features, features);

  // Camera
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  camera_params_t cam_params;
  camera_params_setup(&cam_params, 0, cam_res, pmodel, dmodel, cam_vec);

  // Camera Extrinsic T_BC0
  const real_t cam_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam_ext_r[3] = {0.05, 0.0, 0.0};
  TF_ER(cam_ext_ypr, cam_ext_r, T_BC0);
  TF_VECTOR(T_BC0, cam_ext);

  // Simulate camera trajectory
  sim_circle_t conf;
  sim_circle_defaults(&conf);
  sim_camera_data_t *cam_data = sim_camera_circle_trajectory(&conf,
                                                             T_BC0,
                                                             &cam_params,
                                                             features,
                                                             num_features);
  // ASSERT
  for (size_t k = 0; k < cam_data->num_frames; k++) {
    const sim_camera_frame_t *cam_frame = cam_data->frames[k];
    const real_t *cam_pose = &cam_data->poses[k * 7];

    for (int i = 0; i < cam_frame->n; i++) {
      const size_t feature_id = cam_frame->feature_ids[i];
      const real_t *p_W = &features[feature_id * 3];
      const real_t *z = &cam_frame->keypoints[i * 2];

      TF(cam_pose, T_WC0);
      TF_INV(T_WC0, T_C0W);
      TF_POINT(T_C0W, p_W, p_C0);

      real_t zhat[2] = {0};
      pinhole_radtan4_project(cam_vec, p_C0, zhat);

      const real_t r[2] = {zhat[0] - z[0], zhat[1] - z[1]};
      MU_ASSERT(fltcmp(r[0], 0.0) == 0);
      MU_ASSERT(fltcmp(r[1], 0.0) == 0);
    }
  }

  // Clean up
  sim_camera_data_free(cam_data);

  return 0;
}

// int test_sim_gimbal_malloc_free(void) {
//   sim_gimbal_t *sim = sim_gimbal_malloc();
//   sim_gimbal_free(sim);
//   return 0;
// }
//
// int test_sim_gimbal_view(void) {
//   sim_gimbal_t *sim = sim_gimbal_malloc();
//
//   const timestamp_t ts = 0;
//   const int view_idx = 0;
//   const int cam_idx = 0;
//   real_t pose[7] = {0, 0, 0, 1, 0, 0, 0};
//
//   sim_gimbal_view_t *view = sim_gimbal_view(sim, ts, view_idx, cam_idx, pose);
//   sim_gimbal_view_free(view);
//
//   sim_gimbal_free(sim);
//   return 0;
// }
//
// int test_sim_gimbal_solve(void) {
//   // Setup gimbal simulator
//   sim_gimbal_t *sim = sim_gimbal_malloc();
//
//   // Setup gimbal calibrator
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//   const timestamp_t ts = 0;
//   calib_gimbal_add_fiducial(calib, sim->fiducial_ext.data);
//   calib_gimbal_add_pose(calib, ts, sim->gimbal_pose.data);
//   calib_gimbal_add_gimbal_extrinsic(calib, sim->gimbal_ext.data);
//   calib_gimbal_add_gimbal_link(calib, 0, sim->gimbal_links[0].data);
//   calib_gimbal_add_gimbal_link(calib, 1, sim->gimbal_links[1].data);
//   for (int cam_idx = 0; cam_idx < sim->num_cams; cam_idx++) {
//     calib_gimbal_add_camera(calib,
//                             cam_idx,
//                             sim->cam_params[cam_idx].resolution,
//                             sim->cam_params[cam_idx].proj_model,
//                             sim->cam_params[cam_idx].dist_model,
//                             sim->cam_params[cam_idx].data,
//                             sim->cam_exts[cam_idx].data);
//   }
//
//   // Setup solver
//   solver_t solver;
//   solver_setup(&solver);
//   solver.verbose = 1;
//   solver.param_order_func = &calib_gimbal_param_order;
//   solver.cost_func = &calib_gimbal_cost;
//   solver.linearize_func = &calib_gimbal_linearize_compact;
//
//   // Simulate gimbal views
//   int num_views = 100;
//   int num_cams = 2;
//   const int pose_idx = 0;
//   sim_gimbal_view_t *view = NULL;
//
//   for (int view_idx = 0; view_idx < num_views; view_idx++) {
//     // Add gimbal view
//     for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
//       // Simulate single gimbal view
//       const timestamp_t ts = view_idx;
//       view = sim_gimbal_view(sim, ts, view_idx, cam_idx, sim->gimbal_pose.data);
//
//       // Add view to calibration problem
//       real_t joints[3] = {0};
//       sim_gimbal_get_joints(sim, 3, joints);
//       calib_gimbal_add_view(calib,
//                             pose_idx,
//                             view_idx,
//                             view_idx,
//                             cam_idx,
//                             view->num_measurements,
//                             view->tag_ids,
//                             view->corner_indices,
//                             view->object_points,
//                             view->keypoints,
//                             joints,
//                             sim->num_joints);
//       sim_gimbal_view_free(view);
//     }
//
//     // Find gimbal NBV
//     // real_t nbv_joints[3] = {0};
//     // calib_gimbal_nbv(calib, nbv_joints);
//     // sim_gimbal_set_joint(sim, 0, nbv_joints[0]);
//     // sim_gimbal_set_joint(sim, 1, nbv_joints[1]);
//     // sim_gimbal_set_joint(sim, 2, nbv_joints[2]);
//   }
//
//   // Solve
//   solver_solve(&solver, calib);
//
//   // Clean up
//   calib_gimbal_free(calib);
//   sim_gimbal_free(sim);
//
//   return 0;
// }

void test_suite(void) {
  // SIM
  MU_ADD_TEST(test_sim_features_load);
  MU_ADD_TEST(test_sim_imu_data_load);
  MU_ADD_TEST(test_sim_camera_frame_load);
  MU_ADD_TEST(test_sim_camera_data_load);
  MU_ADD_TEST(test_sim_camera_circle_trajectory);
  // MU_ADD_TEST(test_sim_gimbal_malloc_free);
  // MU_ADD_TEST(test_sim_gimbal_view);
  // MU_ADD_TEST(test_sim_gimbal_solve);
}
MU_RUN_TESTS(test_suite)
