#include "munit.h"
#include "xyz.h"
#include "xyz_calib.h"
#include "xyz_aprilgrid.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_CAM_APRIL TEST_DATA_PATH "cam_april"
#define TEST_IMU_APRIL TEST_DATA_PATH "imu_april"

int test_camchain(void) {
  // Form camera poses
  int num_cams = 5;
  real_t T_C0F[4 * 4] = {0};
  real_t T_C1F[4 * 4] = {0};
  real_t T_C2F[4 * 4] = {0};
  real_t T_C3F[4 * 4] = {0};
  real_t T_C4F[4 * 4] = {0};
  real_t *poses[5] = {T_C0F, T_C1F, T_C2F, T_C3F, T_C4F};

  for (int k = 0; k < num_cams; k++) {
    const real_t ypr[3] = {randf(-90, 90), randf(-90, 90), randf(-90, 90)};
    const real_t r[3] = {randf(-1, 1), randf(-1, 1), randf(-1, 1)};
    tf_er(ypr, r, poses[k]);
  }

  TF_INV(T_C0F, T_FC0);
  TF_INV(T_C1F, T_FC1);
  TF_INV(T_C2F, T_FC2);
  TF_INV(T_C3F, T_FC3);
  TF_INV(T_C4F, T_FC4);
  real_t *poses_inv[5] = {T_FC0, T_FC1, T_FC2, T_FC3, T_FC4};

  // Camchain
  camchain_t *camchain = camchain_malloc(num_cams);
  camchain_add_pose(camchain, 0, 0, T_C0F);
  camchain_add_pose(camchain, 1, 0, T_C1F);
  camchain_add_pose(camchain, 2, 0, T_C2F);
  camchain_add_pose(camchain, 3, 0, T_C3F);
  camchain_adjacency(camchain);
  // camchain_adjacency_print(camchain);

  for (int cam_i = 1; cam_i < num_cams; cam_i++) {
    for (int cam_j = 1; cam_j < num_cams; cam_j++) {
      // Get ground-truth
      TF_CHAIN(T_CiCj_gnd, 2, poses[cam_i], poses_inv[cam_j]);

      // Get camchain result
      real_t T_CiCj_est[4 * 4] = {0};
      int status = camchain_find(camchain, cam_i, cam_j, T_CiCj_est);

      if (cam_i != 4 && cam_j != 4) { // Camera 4 was not added
        MU_ASSERT(status == 0);
      } else {
        MU_ASSERT(status == -1);
      }
    }
  }

  // Clean up
  camchain_free(camchain);

  return 0;
}

typedef struct test_calib_camera_data_t {
  real_t T_WF[4 * 4];
  real_t T_WB[4 * 4];
  real_t T_BF[4 * 4];
  real_t T_BCi[4 * 4];

  pose_t fiducial;     // T_WF
  pose_t pose;         // T_WB
  pose_t rel_pose;     // T_BF
  extrinsic_t cam_ext; // T_BCi
  camera_params_t cam_params;

  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];
} test_calib_camera_data_t;

void test_calib_camera_data_setup(test_calib_camera_data_t *data) {
  // Calibration target pose T_WF
  real_t fiducial_data[7] = {0};
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.01, 0.01, 0.01};
  tf_er(ypr_WF, r_WF, data->T_WF);
  tf_vector(data->T_WF, fiducial_data);
  pose_setup(&data->fiducial, 0, fiducial_data);

  // Body pose T_WB
  real_t pose_data[7] = {0};
  real_t ypr_WB[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  real_t r_WB[3] = {-10.0, 0.001, 0.001};
  tf_er(ypr_WB, r_WB, data->T_WB);
  tf_vector(data->T_BF, pose_data);
  pose_setup(&data->pose, 0, pose_data);

  // Relative pose T_BF
  real_t rel_pose_data[7] = {0};
  TF_INV(data->T_WB, T_BW);
  tf_chain2(2, T_BW, data->T_WF, data->T_BF);
  tf_vector(data->T_BF, rel_pose_data);
  pose_setup(&data->rel_pose, 0, rel_pose_data);

  // Camera extrinsics T_BCi
  real_t cam_ext_data[7] = {0};
  real_t ypr_BCi[3] = {0.01, 0.01, 0.0};
  real_t r_BCi[3] = {0.001, 0.001, 0.001};
  tf_er(ypr_BCi, r_BCi, data->T_BCi);
  tf_vector(data->T_BCi, cam_ext_data);
  extrinsic_setup(&data->cam_ext, cam_ext_data);

  // Camera
  data->cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  camera_params_setup(&data->cam_params,
                      data->cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_data);

  // Project to image plane
  int num_rows = 6;
  int num_cols = 6;
  double tag_size = 0.088;
  double tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  data->tag_id = 1;
  data->corner_idx = 2;
  aprilgrid_object_point(grid, data->tag_id, data->corner_idx, data->p_FFi);

  TF_INV(data->T_BCi, T_CiB);
  TF_CHAIN(T_CiF, 2, T_CiB, data->T_BF);
  TF_POINT(T_CiF, data->p_FFi, p_CiFi);
  pinhole_radtan4_project(cam_data, p_CiFi, data->z);

  aprilgrid_free(grid);
}

int test_calib_camera_factor(void) {
  // Setup
  test_calib_camera_data_t calib_data;
  test_calib_camera_data_setup(&calib_data);

  calib_camera_factor_t factor;
  const real_t var[2] = {1.0, 1.0};
  calib_camera_factor_setup(&factor,
                            &calib_data.rel_pose,
                            &calib_data.cam_ext,
                            &calib_data.cam_params,
                            calib_data.cam_idx,
                            calib_data.tag_id,
                            calib_data.corner_idx,
                            calib_data.p_FFi,
                            calib_data.z,
                            var);

  // Evaluate
  calib_camera_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  CHECK_FACTOR_J(0, factor, calib_camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, calib_camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, calib_camera_factor_eval, step_size, tol, 0);

  return 0;
}

typedef struct test_calib_imucam_data_t {
  real_t T_WF[4 * 4];
  real_t T_WS[4 * 4];
  real_t T_SC0[4 * 4];
  real_t T_C0Ci[4 * 4];

  fiducial_t fiducial; // T_WF
  pose_t imu_pose;     // T_WB
  extrinsic_t imu_ext; // T_SC0
  extrinsic_t cam_ext; // T_C0Ci
  camera_params_t cam_params;
  time_delay_t time_delay;

  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];
} test_calib_imucam_data_t;

void test_calib_imucam_data_setup(test_calib_imucam_data_t *data) {
  // Calibration target pose T_WF
  real_t fiducial_data[7] = {0};
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.01, 0.01, 0.01};
  tf_er(ypr_WF, r_WF, data->T_WF);
  tf_vector(data->T_WF, fiducial_data);
  fiducial_setup(&data->fiducial, fiducial_data);

  // IMU pose T_WS
  real_t imu_pose_data[7] = {0};
  real_t ypr_WS[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  real_t r_WS[3] = {-10.0, 0.001, 0.001};
  tf_er(ypr_WS, r_WS, data->T_WS);
  tf_vector(data->T_WS, imu_pose_data);
  pose_setup(&data->imu_pose, 0, imu_pose_data);

  // IMU extrinsics T_SC0
  real_t imu_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  tf(imu_ext_data, data->T_SC0);
  extrinsic_setup(&data->imu_ext, imu_ext_data);

  // Camera extrinsics T_C0Ci
  real_t cam_ext_data[7] = {0};
  real_t ypr_C0Ci[3] = {0.01, 0.01, 0.0};
  real_t r_C0Ci[3] = {0.001, 0.001, 0.001};
  tf_er(ypr_C0Ci, r_C0Ci, data->T_C0Ci);
  tf_vector(data->T_C0Ci, cam_ext_data);
  extrinsic_setup(&data->cam_ext, cam_ext_data);

  // Camera
  data->cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  camera_params_setup(&data->cam_params,
                      data->cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_data);

  // Time delay
  time_delay_setup(&data->time_delay, 0.0);

  // Project to image plane
  int num_rows = 6;
  int num_cols = 6;
  double tag_size = 0.088;
  double tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  data->tag_id = 1;
  data->corner_idx = 2;
  aprilgrid_object_point(grid, data->tag_id, data->corner_idx, data->p_FFi);

  TF_INV(data->T_WS, T_SW);
  TF_INV(data->T_SC0, T_C0S);
  TF_INV(data->T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiF, 4, T_CiC0, T_C0S, T_SW, data->T_WF);
  TF_POINT(T_CiF, data->p_FFi, p_CiFi);
  pinhole_radtan4_project(cam_data, p_CiFi, data->z);

  aprilgrid_free(grid);
}

int test_calib_imucam_factor(void) {
  // Setup
  test_calib_imucam_data_t calib_data;
  test_calib_imucam_data_setup(&calib_data);

  calib_imucam_factor_t factor;
  const real_t var[2] = {1.0, 1.0};
  const real_t v[2] = {0.01, 0.02};
  calib_imucam_factor_setup(&factor,
                            &calib_data.fiducial,
                            &calib_data.imu_pose,
                            &calib_data.imu_ext,
                            &calib_data.cam_ext,
                            &calib_data.cam_params,
                            &calib_data.time_delay,
                            calib_data.cam_idx,
                            calib_data.tag_id,
                            calib_data.corner_idx,
                            calib_data.p_FFi,
                            calib_data.z,
                            v,
                            var);

  // Evaluate
  calib_imucam_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-2;
  const double step_size = 1e-8;
  const int debug = 0;
  CHECK_FACTOR_J(0, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(1, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(2, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(3, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(4, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(5, factor, calib_imucam_factor_eval, step_size, tol, debug);

  return 0;
}

// int test_calib_camera_mono_batch(void) {
  // const char *data_path = TEST_CAM_APRIL "/cam0";
  //
  // // Initialize camera intrinsics
  // const int cam_res[2] = {752, 480};
  // const char *proj_model = "pinhole";
  // const char *dist_model = "radtan4";
  // const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  // const real_t cam_params[8] =
  //     {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};

  // // Setup camera calibration problem
  // calib_camera_t *calib = calib_camera_malloc();
  // calib->verbose = 0;
  // calib->max_iter = 30;
  // calib_camera_add_camera(calib,
  //                         0,
  //                         cam_res,
  //                         proj_model,
  //                         dist_model,
  //                         cam_params,
  //                         cam_ext);
  //
  // // Batch solve
  // calib_camera_add_data(calib, 0, data_path);
  // calib_camera_solve(calib);
  //
  // // Asserts
  // double reproj_rmse = 0.0;
  // double reproj_mean = 0.0;
  // double reproj_median = 0.0;
  // calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
  // MU_ASSERT(reproj_rmse < 0.5);
  // MU_ASSERT(reproj_mean < 0.5);
  // MU_ASSERT(reproj_median < 0.5);
  //
  // // Clean up
  // calib_camera_free(calib);

//   return 0;
// }

// int test_calib_camera_mono_ceres(void) {
//   const char *data_path = TEST_CAM_APRIL "/cam0";
//
//   // Initialize camera intrinsics
//   const int cam_res[2] = {752, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   const real_t cam_params[8] =
//       {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};
//
//   // Setup camera calibration problem
//   calib_camera_t *calib = calib_camera_malloc();
//   calib->verbose = 0;
//   calib_camera_add_camera(calib,
//                           0,
//                           cam_res,
//                           proj_model,
//                           dist_model,
//                           cam_params,
//                           cam_ext);
//
//   // Batch solve
//   calib_camera_add_data(calib, 0, data_path);
//
//   // Setup solver
//   ceres_init();
//   ceres_problem_t *problem = ceres_create_problem();
//   ceres_local_parameterization_t *pose_pm =
//       ceres_create_pose_local_parameterization();
//
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_camera_factor_t *factor = &view->factors[factor_idx];
//         real_t **param_ptrs = factor->params;
//         int num_residuals = 2;
//         int num_params = 3;
//         int param_sizes[3] = {
//             7, // Pose
//             7, // Camera extrinsic
//             8, // Camera parameters
//         };
//         ceres_problem_add_residual_block(problem,
//                                          &calib_camera_factor_ceres_eval,
//                                          factor,
//                                          NULL,
//                                          NULL,
//                                          num_residuals,
//                                          num_params,
//                                          param_sizes,
//                                          param_ptrs);
//
//         ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
//         ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // Solve
//   // ceres_solve(problem, 20, 0);
//   ceres_solve(problem);
//   // calib_camera_print(calib);
//
//   // Asserts
//   double reproj_rmse = 0.0;
//   double reproj_mean = 0.0;
//   double reproj_median = 0.0;
//   calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
//   MU_ASSERT(reproj_rmse < 0.5);
//   MU_ASSERT(reproj_mean < 0.5);
//   MU_ASSERT(reproj_median < 0.5);
//
//   // Clean up
//   calib_camera_free(calib);
//   ceres_free_problem(problem);
//
//   return 0;
// }
//
// int test_calib_camera_mono_incremental(void) {
//   const char *data_path = TEST_CAM_APRIL "/cam0";
//
//   // Initialize camera intrinsics
//   const int res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   const real_t cam_vec[8] =
//       {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};
//
//   // Setup camera calibration problem
//   calib_camera_t *calib = calib_camera_malloc();
//   calib->verbose = 0;
//   calib_camera_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);
//
//   // Incremental solve
//   int window_size = 2;
//   int cam_idx = 0;
//   int num_files = 0;
//   char **files = list_files(data_path, &num_files);
//
//   calib->verbose = 0;
//   // TIC(calib_camera_loop);
//
//   for (int view_idx = 0; view_idx < num_files; view_idx++) {
//     // Load aprilgrid
//     aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
//
//     // Get aprilgrid measurements
//     const timestamp_t ts = grid->timestamp;
//     const int num_corners = grid->corners_detected;
//     int *tag_ids = MALLOC(int, num_corners);
//     int *corner_indices = MALLOC(int, num_corners);
//     real_t *kps = MALLOC(real_t, num_corners * 2);
//     real_t *pts = MALLOC(real_t, num_corners * 3);
//     aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);
//
//     // Add view
//     calib_camera_add_view(calib,
//                           ts,
//                           view_idx,
//                           cam_idx,
//                           num_corners,
//                           tag_ids,
//                           corner_indices,
//                           pts,
//                           kps);
//
//     // Incremental solve
//     if (calib->num_views >= window_size) {
//       calib_camera_marginalize(calib);
//     }
//     calib_camera_solve(calib);
//
//     // Clean up
//     free(tag_ids);
//     free(corner_indices);
//     free(kps);
//     free(pts);
//     aprilgrid_free(grid);
//   }
//
//   // calib_camera_print(calib);
//   // const real_t time_taken = TOC(calib_camera_loop);
//   // const real_t rate_hz = num_files / time_taken;
//   // printf("%d frames in %.2f [s] or %.2f Hz\n", num_files, time_taken, rate_hz);
//
//   // Clean up
//   for (int view_idx = 0; view_idx < num_files; view_idx++) {
//     free(files[view_idx]);
//   }
//   free(files);
//   calib_camera_free(calib);
//
//   return 0;
// }
//
// int test_calib_camera_stereo_batch(void) {
//   // Initialize camera intrinsics
//   int num_cams = 2;
//   char *data_dir = TEST_CAM_APRIL "/cam%d";
//   const int cam_res[2] = {752, 480};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   const real_t focal = pinhole_focal(cam_res[0], 90.0);
//   const real_t cx = cam_res[0] / 2.0;
//   const real_t cy = cam_res[1] / 2.0;
//   const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   real_t cam[2][8] = {{focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0},
//                       {focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0}};
//
//   camera_params_t cam_params[2];
//   camera_params_setup(&cam_params[0], 0, cam_res, pmodel, dmodel, cam[0]);
//   camera_params_setup(&cam_params[1], 1, cam_res, pmodel, dmodel, cam[1]);
//
//   for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
//     char data_path[1024] = {0};
//     sprintf(data_path, data_dir, cam_idx);
//
//     calib_camera_t *cam_calib = calib_camera_malloc();
//     cam_calib->verbose = 0;
//     calib_camera_add_camera(cam_calib,
//                             0,
//                             cam_res,
//                             pmodel,
//                             dmodel,
//                             cam[cam_idx],
//                             cam_ext);
//     calib_camera_add_data(cam_calib, 0, data_path);
//     calib_camera_solve(cam_calib);
//     vec_copy(cam_calib->cam_params[0].data, 8, cam_params[cam_idx].data);
//     calib_camera_free(cam_calib);
//   }
//
//   // Initialize camera extrinsics
//   camchain_t *camchain = camchain_malloc(num_cams);
//   for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
//     char data_path[1024] = {0};
//     sprintf(data_path, data_dir, cam_idx);
//
//     // Get camera data
//     int num_files = 0;
//     char **files = list_files(data_path, &num_files);
//
//     // Exit if no calibration data
//     if (num_files == 0) {
//       for (int view_idx = 0; view_idx < num_files; view_idx++) {
//         free(files[view_idx]);
//       }
//       free(files);
//       return -1;
//     }
//
//     for (int view_idx = 0; view_idx < num_files; view_idx++) {
//       // Load aprilgrid
//       aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
//       if (grid->corners_detected == 0) {
//         free(files[view_idx]);
//         aprilgrid_free(grid);
//         continue;
//       }
//
//       // Get aprilgrid measurements
//       const timestamp_t ts = grid->timestamp;
//       const int n = grid->corners_detected;
//       int *tag_ids = MALLOC(int, n);
//       int *corner_indices = MALLOC(int, n);
//       real_t *kps = MALLOC(real_t, n * 2);
//       real_t *pts = MALLOC(real_t, n * 3);
//       aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);
//
//       // Estimate relative pose T_CiF and add to camchain
//       real_t T_CiF[4 * 4] = {0};
//       if (solvepnp_camera(&cam_params[cam_idx], kps, pts, n, T_CiF) == 0) {
//         camchain_add_pose(camchain, cam_idx, ts, T_CiF);
//       }
//
//       // Clean up
//       free(tag_ids);
//       free(corner_indices);
//       free(kps);
//       free(pts);
//       aprilgrid_free(grid);
//       free(files[view_idx]);
//     }
//     free(files);
//   }
//   camchain_adjacency(camchain);
//   real_t T_CiCj[4 * 4] = {0};
//   camchain_find(camchain, 0, 1, T_CiCj);
//   camchain_free(camchain);
//
//   // Setup Camera calibrator
//   const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   TF_VECTOR(T_CiCj, cam1_ext);
//
//   calib_camera_t *stereo_calib = calib_camera_malloc();
//   stereo_calib->verbose = 0;
//   stereo_calib->max_iter = 50;
//   calib_camera_add_camera(stereo_calib,
//                           0,
//                           cam_res,
//                           pmodel,
//                           dmodel,
//                           cam_params[0].data,
//                           cam0_ext);
//   calib_camera_add_camera(stereo_calib,
//                           1,
//                           cam_res,
//                           pmodel,
//                           dmodel,
//                           cam_params[1].data,
//                           cam1_ext);
//   for (int cam_idx = 0; cam_idx < stereo_calib->num_cams; cam_idx++) {
//     char data_path[1024] = {0};
//     sprintf(data_path, data_dir, cam_idx);
//     calib_camera_add_data(stereo_calib, cam_idx, data_path);
//   }
//   calib_camera_solve(stereo_calib);
//   // calib_camera_print(stereo_calib);
//
//   // Asserts
//   double reproj_rmse = 0.0;
//   double reproj_mean = 0.0;
//   double reproj_median = 0.0;
//   calib_camera_errors(stereo_calib, &reproj_rmse, &reproj_mean, &reproj_median);
//   MU_ASSERT(reproj_rmse < 0.5);
//   MU_ASSERT(reproj_mean < 0.5);
//   MU_ASSERT(reproj_median < 0.5);
//
//   // Clean up
//   calib_camera_free(stereo_calib);
//
//   return 0;
// }
//
// int test_calib_camera_stereo_ceres(void) {
//   // Initialize camera intrinsics
//   int num_cams = 2;
//   char *data_dir = TEST_CAM_APRIL "/cam%d";
//   const int cam_res[2] = {752, 480};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   const real_t focal = pinhole_focal(cam_res[0], 90.0);
//   const real_t cx = cam_res[0] / 2.0;
//   const real_t cy = cam_res[1] / 2.0;
//   real_t cam[2][8] = {{focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0},
//                       {focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0}};
//   camera_params_t cam_params[2];
//   camera_params_setup(&cam_params[0], 0, cam_res, pmodel, dmodel, cam[0]);
//   camera_params_setup(&cam_params[1], 1, cam_res, pmodel, dmodel, cam[1]);
//
//   // Initialize camera extrinsics
//   camchain_t *camchain = camchain_malloc(num_cams);
//   for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
//     char data_path[1024] = {0};
//     sprintf(data_path, data_dir, cam_idx);
//
//     // Get camera data
//     int num_files = 0;
//     char **files = list_files(data_path, &num_files);
//
//     // Exit if no calibration data
//     if (num_files == 0) {
//       for (int view_idx = 0; view_idx < num_files; view_idx++) {
//         free(files[view_idx]);
//       }
//       free(files);
//       return -1;
//     }
//
//     for (int view_idx = 0; view_idx < num_files; view_idx++) {
//       // Load aprilgrid
//       aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
//       if (grid->corners_detected == 0) {
//         free(files[view_idx]);
//         aprilgrid_free(grid);
//         continue;
//       }
//
//       // Get aprilgrid measurements
//       const timestamp_t ts = grid->timestamp;
//       const int n = grid->corners_detected;
//       int *tag_ids = MALLOC(int, n);
//       int *corner_indices = MALLOC(int, n);
//       real_t *kps = MALLOC(real_t, n * 2);
//       real_t *pts = MALLOC(real_t, n * 3);
//       aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);
//
//       // Estimate relative pose T_CiF and add to camchain
//       real_t T_CiF[4 * 4] = {0};
//       if (solvepnp_camera(&cam_params[cam_idx], kps, pts, n, T_CiF) == 0) {
//         camchain_add_pose(camchain, cam_idx, ts, T_CiF);
//       }
//
//       // Clean up
//       free(tag_ids);
//       free(corner_indices);
//       free(kps);
//       free(pts);
//       aprilgrid_free(grid);
//       free(files[view_idx]);
//     }
//     free(files);
//   }
//   camchain_adjacency(camchain);
//   real_t T_CiCj[4 * 4] = {0};
//   camchain_find(camchain, 0, 1, T_CiCj);
//   camchain_free(camchain);
//
//   // Setup Camera calibrator
//   const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   TF_VECTOR(T_CiCj, cam1_ext);
//
//   calib_camera_t *calib = calib_camera_malloc();
//   calib->verbose = 0;
//   calib_camera_add_camera(calib,
//                           0,
//                           cam_res,
//                           pmodel,
//                           dmodel,
//                           cam_params[0].data,
//                           cam0_ext);
//   calib_camera_add_camera(calib,
//                           1,
//                           cam_res,
//                           pmodel,
//                           dmodel,
//                           cam_params[1].data,
//                           cam1_ext);
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     char data_path[1024] = {0};
//     sprintf(data_path, data_dir, cam_idx);
//     calib_camera_add_data(calib, cam_idx, data_path);
//   }
//   // calib_camera_solve(calib);
//
//   // Setup solver
//   ceres_init();
//   ceres_problem_t *problem = ceres_create_problem();
//   ceres_local_parameterization_t *pose_pm =
//       ceres_create_pose_local_parameterization();
//
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_camera_factor_t *factor = &view->factors[factor_idx];
//         real_t **param_ptrs = factor->params;
//         int num_residuals = 2;
//         int num_params = 3;
//         int param_sizes[3] = {
//             7, // Pose
//             7, // Camera extrinsic
//             8, // Camera parameters
//         };
//         ceres_problem_add_residual_block(problem,
//                                          &calib_camera_factor_ceres_eval,
//                                          factor,
//                                          NULL,
//                                          NULL,
//                                          num_residuals,
//                                          num_params,
//                                          param_sizes,
//                                          param_ptrs);
//
//         ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
//         ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // Solve
//   // ceres_solve(problem, 20, 0);
//   ceres_solve(problem);
//   // calib_camera_print(calib);
//
//   // Asserts
//   double reproj_rmse = 0.0;
//   double reproj_mean = 0.0;
//   double reproj_median = 0.0;
//   calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
//   MU_ASSERT(reproj_rmse < 0.5);
//   MU_ASSERT(reproj_mean < 0.5);
//   MU_ASSERT(reproj_median < 0.5);
//
//   // Clean up
//   calib_camera_free(calib);
//   ceres_free_problem(problem);
//
//   return 0;
// }
//
// int test_calib_imucam_view(void) {
//   // Setup Camera
//   const int cam_res[2] = {752, 480};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
//   const real_t cam_ext_vec[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//
//   camera_params_t camera_params;
//   extrinsic_t camera_extrinsic;
//   camera_params_setup(&camera_params, 0, cam_res, pmodel, dmodel, cam_vec);
//   extrinsic_setup(&camera_extrinsic, cam_ext_vec);
//
//   // Fiducial
//   fiducial_t fiducial;
//   const real_t fiducial_vec[7] = {1.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5};
//   fiducial_setup(&fiducial, fiducial_vec);
//
//   // IMU pose
//   const timestamp_t ts = 0;
//   const real_t pose_vec[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_t imu_pose;
//   pose_setup(&imu_pose, ts, pose_vec);
//
//   // IMU extrinsic
//   const real_t imu_ext_vec[7] = {0.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_t imu_extrinsic;
//   extrinsic_setup(&imu_extrinsic, imu_ext_vec);
//
//   // Time delay
//   time_delay_t time_delay;
//   time_delay_setup(&time_delay, 0.0);
//
//   // Create calib imucam view
//   const int view_idx = 0;
//   const int cam_idx = 0;
//   const int num_corners = 1;
//   const int tag_ids[1] = {0};
//   const int corner_indices[1] = {0};
//   const real_t object_points[3] = {
//       0.0,
//       0.0,
//       0.0,
//   };
//
//   // -- Transform fiducial point from fiducial frame to camera frame
//   const real_t p_F[3] = {object_points[0], object_points[1], object_points[2]};
//   TF(fiducial_vec, T_WF);
//   TF(pose_vec, T_WS);
//   TF(imu_ext_vec, T_BS);
//   TF(cam_ext_vec, T_BC0);
//   TF_INV(T_WS, T_SW);
//   TF_INV(T_BC0, T_C0B);
//   TF_CHAIN(T_C0F, 2, T_C0B, T_BS, T_SW, T_WF);
//   TF_POINT(T_C0F, p_F, p_C0);
//
//   // -- Project keypoint
//   real_t keypoints[2] = {0};
//   pinhole_radtan4_project(cam_vec, p_C0, keypoints);
//
//   // -- Create view
//   calib_imucam_view_t *view = calib_imucam_view_malloc(ts,
//                                                        view_idx,
//                                                        cam_idx,
//                                                        num_corners,
//                                                        tag_ids,
//                                                        corner_indices,
//                                                        object_points,
//                                                        keypoints,
//                                                        &fiducial,
//                                                        &imu_pose,
//                                                        &imu_extrinsic,
//                                                        &camera_extrinsic,
//                                                        &camera_params,
//                                                        &time_delay);
//
//   // Clean up
//   calib_imucam_view_free(view);
//
//   return 0;
// }
//
// int test_calib_imucam_add_imu(void) {
//   // Setup
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//
//   calib_imucam_t *calib = calib_imucam_malloc();
//   calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
//
//   // Assert
//   MU_ASSERT(calib->imu_params.rate == imu_rate);
//   MU_ASSERT(fltcmp(calib->imu_params.sigma_aw, n_aw) == 0);
//   MU_ASSERT(fltcmp(calib->imu_params.sigma_gw, n_gw) == 0);
//   MU_ASSERT(fltcmp(calib->imu_params.sigma_a, n_a) == 0);
//   MU_ASSERT(fltcmp(calib->imu_params.sigma_g, n_g) == 0);
//   MU_ASSERT(fltcmp(calib->imu_params.g, g) == 0);
//   MU_ASSERT(calib->num_imus == 1);
//
//   // Clean up
//   calib_imucam_free(calib);
//
//   return 0;
// }
//
// int test_calib_imucam_add_camera(void) {
//   // Setup
//   const int res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
//   const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//
//   calib_imucam_t *calib = calib_imucam_malloc();
//   calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);
//
//   // Assert
//   MU_ASSERT(calib->cam_params[0].resolution[0] == 752);
//   MU_ASSERT(calib->cam_params[0].resolution[1] == 480);
//   MU_ASSERT(strcmp(calib->cam_params[0].proj_model, pm) == 0);
//   MU_ASSERT(strcmp(calib->cam_params[0].dist_model, dm) == 0);
//   MU_ASSERT(vec_equals(calib->cam_params[0].data, cam_vec, 8) == 1);
//   MU_ASSERT(vec_equals(calib->cam_exts[0].data, cam_ext, 7) == 1);
//   MU_ASSERT(calib->num_cams == 1);
//
//   // Clean up
//   calib_imucam_free(calib);
//
//   return 0;
// }
//
// int test_calib_imucam_add_imu_event(void) {
//   // Setup
//   calib_imucam_t *calib = calib_imucam_malloc();
//
//   // -- Add imu
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
//   MU_ASSERT(calib->imu_ok == 0);
//   MU_ASSERT(calib->num_imus == 1);
//
//   // -- Add imu event
//   const timestamp_t ts = 1;
//   const real_t acc[3] = {1.0, 2.0, 3.0};
//   const real_t gyr[3] = {4.0, 5.0, 6.0};
//   calib_imucam_add_imu_event(calib, ts, acc, gyr);
//
//   // Assert
//   MU_ASSERT(calib->imu_buf.size == 1);
//   MU_ASSERT(calib->imu_buf.ts[0] == ts);
//   MU_ASSERT(vec_equals(calib->imu_buf.acc[0], acc, 3) == 1);
//   MU_ASSERT(vec_equals(calib->imu_buf.gyr[0], gyr, 3) == 1);
//   MU_ASSERT(calib->imu_ok == 1);
//
//   // Clean up
//   calib_imucam_free(calib);
//
//   return 0;
// }
//
// int test_calib_imucam_add_fiducial_event(void) {
//   // Setup
//   calib_imucam_t *calib = calib_imucam_malloc();
//   // -- Add Imu
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
//   calib->imu_ok = 1;
//   // -- Add camera
//   const int res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
//   const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);
//
//   // Add fiducial event
//   const timestamp_t ts = 1;
//   const int cam_idx = 1;
//   const int n = 2;
//   const int tag_ids[2] = {1, 2};
//   const int corner_idxs[2] = {1, 2};
//   const real_t pts[2 * 3] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
//   const real_t kps[2 * 2] = {0.0, 0.0, 1.0, 1.0};
//   calib_imucam_add_fiducial_event(calib,
//                                   ts,
//                                   cam_idx,
//                                   n,
//                                   tag_ids,
//                                   corner_idxs,
//                                   pts,
//                                   kps);
//
//   // Assert
//   const fiducial_buffer_t *buf = calib->fiducial_buffer;
//   MU_ASSERT(buf->data[0]->ts == ts);
//   MU_ASSERT(buf->data[0]->cam_idx == cam_idx);
//   MU_ASSERT(buf->data[0]->num_corners == n);
//   MU_ASSERT(vec_equals(buf->data[0]->object_points, pts, n * 3) == 1);
//   MU_ASSERT(vec_equals(buf->data[0]->keypoints, kps, n * 2) == 1);
//   MU_ASSERT(buf->size == 1);
//   MU_ASSERT(buf->capacity > 1);
//
//   // Clean up
//   calib_imucam_free(calib);
//
//   return 0;
// }
//
// int test_calib_imucam_update(void) {
//   // Setup
//   calib_imucam_t *calib = calib_imucam_malloc();
//   // -- Add Imu
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 0.70710678, 0.0, 0.0, 0.70710678};
//   calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
//   calib->imu_ok = 1;
//   // -- Add cam0
//   const int cam0_res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam0_vec[8] = {
//       458.654,
//       457.296,
//       367.215,
//       248.375,
//       -0.28340811,
//       0.07395907,
//       0.00019359,
//       1.76187114e-05,
//   };
//   const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   calib_imucam_add_camera(calib, 0, cam0_res, pm, dm, cam0_vec, cam0_ext);
//   // -- Add cam1
//   const int cam1_res[2] = {752, 480};
//   const real_t cam1_vec[8] = {
//       457.587,
//       456.134,
//       379.999,
//       255.238,
//       -0.28368365,
//       0.07451284,
//       -0.00010473,
//       -3.55590700e-05,
//   };
//   const real_t cam1_ext[7] = {
//       0.11007414,
//       -0.00015661,
//       0.00088938,
//       9.99974496e-01,
//       7.04530576e-03,
//       -1.79854893e-04,
//       1.15733025e-03,
//   };
//   calib_imucam_add_camera(calib, 1, cam1_res, pm, dm, cam1_vec, cam1_ext);
//
//   // Test update
//   char *data_dir = TEST_IMU_APRIL;
//   int num_cams = 1;
//   int num_imus = 1;
//   timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
//
//   for (int k = 0; k < timeline->timeline_length; k++) {
//     // Extract timeline events. Add either imu or fiducial event
//     for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
//       timeline_event_t *event = timeline->timeline_events[k][i];
//       const timestamp_t ts = event->ts;
//
//       if (event->type == IMU_EVENT) {
//         const imu_event_t *data = &event->data.imu;
//         calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);
//
//       } else if (event->type == FIDUCIAL_EVENT) {
//         const fiducial_event_t *data = &event->data.fiducial;
//         const int cam_idx = data->cam_idx;
//         calib_imucam_add_fiducial_event(calib,
//                                         ts,
//                                         cam_idx,
//                                         data->num_corners,
//                                         data->tag_ids,
//                                         data->corner_indices,
//                                         data->object_points,
//                                         data->keypoints);
//       }
//     }
//
//     // Trigger update
//     calib_imucam_update(calib);
//     if (calib->num_views == 10) {
//       break;
//     }
//   }
//
//   // Clean up
//   timeline_free(timeline);
//   calib_imucam_free(calib);
//
//   return 0;
// }
//
// int test_calib_imucam_batch(void) {
//   // clang-format off
//   const int res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam_vec[2][8] = {
//     {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05},
//     {457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.555e-05}
//   };
//   const real_t cam_exts[2][7] = {
//     {0, 0, 0, 1, 0, 0, 0},
//     {0.11007414, -0.00015661, 0.00088938,
//      9.99974496e-01, 7.04530576e-03, -1.79854893e-04, 1.15733025e-03}
//   };
//   const real_t T_SC0[4 * 4] = {
//     0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
//     0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
//     -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
//     0.0, 0.0, 0.0, 1.0
//   };
//   TF_VECTOR(T_SC0, imu_ext);
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   // clang-format on
//
//   calib_imucam_t *calib = calib_imucam_malloc();
//   calib_imucam_add_imu(calib, imu_rate, n_a, n_g, n_aw, n_gw, g, imu_ext);
//   calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec[0], cam_exts[0]);
//   calib_imucam_add_camera(calib, 1, res, pm, dm, cam_vec[1], cam_exts[1]);
//
//   // Incremental solve
//   char *data_dir = TEST_IMU_APRIL;
//   int num_cams = 2;
//   int num_imus = 1;
//   // int window_size = 20;
//   timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
//
//   for (int k = 0; k < timeline->timeline_length; k++) {
//     // Extract timeline events
//     for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
//       timeline_event_t *event = timeline->timeline_events[k][i];
//       const timestamp_t ts = event->ts;
//
//       if (event->type == IMU_EVENT) {
//         const imu_event_t *data = &event->data.imu;
//         calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);
//
//       } else if (event->type == FIDUCIAL_EVENT) {
//         const fiducial_event_t *data = &event->data.fiducial;
//         const int cam_idx = data->cam_idx;
//         calib_imucam_add_fiducial_event(calib,
//                                         ts,
//                                         cam_idx,
//                                         data->num_corners,
//                                         data->tag_ids,
//                                         data->corner_indices,
//                                         data->object_points,
//                                         data->keypoints);
//       }
//     }
//
//     // Trigger update
//     // TIC(start);
//     if (calib_imucam_update(calib) == 0) {
//       // // Incremental solve
//       // if (calib->num_views >= window_size) {
//       //   calib->max_iter = 20;
//       //   calib->verbose = 0;
//       //   calib_imucam_solve(calib);
//       //   // calib_imucam_marginalize(calib);
//       //   // k = timeline->timeline_length;
//
//       //   real_t reproj_rmse = 0.0;
//       //   real_t reproj_mean = 0.0;
//       //   real_t reproj_median = 0.0;
//       //   if (calib->num_views) {
//       //     calib_imucam_errors(calib,
//       //                         &reproj_rmse,
//       //                         &reproj_mean,
//       //                         &reproj_median);
//       //   }
//
//       //   char cam0_str[100] = {0};
//       //   char cam1_str[100] = {0};
//       //   char cam_ext_str[100] = {0};
//       //   char imu_ext_str[100] = {0};
//       //   vec2str(calib->cam_params[0].data, 8, cam0_str);
//       //   vec2str(calib->cam_params[1].data, 8, cam1_str);
//       //   vec2str(calib->cam_exts[1].data, 7, cam_ext_str);
//       //   vec2str(calib->imu_ext->data, 7, imu_ext_str);
//       //   printf("cam0:    %s\n", cam0_str);
//       //   printf("cam1:    %s\n", cam1_str);
//       //   printf("cam ext: %s\n", cam_ext_str);
//       //   printf("imu ext: %s\n", imu_ext_str);
//       //   printf("rmse reproj error: %f\n", reproj_rmse);
//       //   printf("\n");
//       // }
//       // PRINT_TOC("time", start);
//     }
//   }
//
//   // Solve
//   calib->max_iter = 10;
//   calib->verbose = 0;
//   calib_imucam_solve(calib);
//   MU_ASSERT((calib->num_cam_factors + calib->num_imu_factors) > 0);
//
//   // Clean up
//   calib_imucam_free(calib);
//   timeline_free(timeline);
//
//   return 0;
// }
//
// int test_calib_imucam_batch_ceres(void) {
//   // Setup
//   calib_imucam_t *calib = calib_imucam_malloc();
//   // -- Add Imu
//   const int imu_rate = 200;
//   const real_t n_a = 0.08;
//   const real_t n_g = 0.004;
//   const real_t n_aw = 0.00004;
//   const real_t n_gw = 2.0e-6;
//   const real_t g = 9.81;
//   // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 0.70710678, 0.0, 0.0, 0.70710678};
//   const real_t T_SC0[4 * 4] = {0.0148655429818,
//                                -0.999880929698,
//                                0.00414029679422,
//                                -0.0216401454975,
//                                0.999557249008,
//                                0.0149672133247,
//                                0.025715529948,
//                                -0.064676986768,
//                                -0.0257744366974,
//                                0.00375618835797,
//                                0.999660727178,
//                                0.00981073058949,
//                                0.0,
//                                0.0,
//                                0.0,
//                                1.0};
//   TF_VECTOR(T_SC0, imu_ext);
//   calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
//   calib->imu_ok = 1;
//   // -- Add cam0
//   const int cam0_res[2] = {752, 480};
//   const char *pm = "pinhole";
//   const char *dm = "radtan4";
//   const real_t cam0_vec[8] = {
//       458.654,
//       457.296,
//       367.215,
//       248.375,
//       -0.28340811,
//       0.07395907,
//       0.00019359,
//       1.76187114e-05,
//   };
//   const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   calib_imucam_add_camera(calib, 0, cam0_res, pm, dm, cam0_vec, cam0_ext);
//   // -- Add cam1
//   const int cam1_res[2] = {752, 480};
//   const real_t cam1_vec[8] = {
//       457.587,
//       456.134,
//       379.999,
//       255.238,
//       -0.28368365,
//       0.07451284,
//       -0.00010473,
//       -3.55590700e-05,
//   };
//   const real_t cam1_ext[7] = {
//       0.11007414,
//       -0.00015661,
//       0.00088938,
//       9.99974496e-01,
//       7.04530576e-03,
//       -1.79854893e-04,
//       1.15733025e-03,
//   };
//   calib_imucam_add_camera(calib, 1, cam1_res, pm, dm, cam1_vec, cam1_ext);
//
//   // Test update
//   char *data_dir = TEST_IMU_APRIL;
//   int num_cams = 2;
//   int num_imus = 1;
//   timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
//
//   for (int k = 0; k < timeline->timeline_length; k++) {
//     // Extract timeline events. Add either imu or fiducial event
//     for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
//       timeline_event_t *event = timeline->timeline_events[k][i];
//       const timestamp_t ts = event->ts;
//
//       if (event->type == IMU_EVENT) {
//         const imu_event_t *data = &event->data.imu;
//         calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);
//
//       } else if (event->type == FIDUCIAL_EVENT) {
//         const fiducial_event_t *data = &event->data.fiducial;
//         const int cam_idx = data->cam_idx;
//         calib_imucam_add_fiducial_event(calib,
//                                         ts,
//                                         cam_idx,
//                                         data->num_corners,
//                                         data->tag_ids,
//                                         data->corner_indices,
//                                         data->object_points,
//                                         data->keypoints);
//       }
//     }
//
//     // Trigger update
//     calib_imucam_update(calib);
//   }
//
//   // Setup ceres-solver
//   ceres_init();
//   ceres_problem_t *problem = ceres_create_problem();
//   ceres_local_parameterization_t *pose_pm =
//       ceres_create_pose_local_parameterization();
//   int num_factors = 0;
//
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[k];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         real_t **param_ptrs = factor->params;
//         int num_residuals = 2;
//         int num_params = 6;
//         int param_sizes[6] = {
//             7, // Fiducial extrinsic
//             7, // Imu pose
//             7, // Imu extrinsic
//             7, // Camera extrinsic
//             8, // Camera parameters
//             1, // Time delay
//         };
//         ceres_problem_add_residual_block(problem,
//                                          &calib_imucam_factor_ceres_eval,
//                                          factor,
//                                          NULL,
//                                          NULL,
//                                          num_residuals,
//                                          num_params,
//                                          param_sizes,
//                                          param_ptrs);
//         num_factors++;
//
//         ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
//         ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
//         ceres_set_parameterization(problem, param_ptrs[2], pose_pm);
//         ceres_set_parameterization(problem, param_ptrs[3], pose_pm);
//       }
//     }
//   }
//
//   for (int k = 0; k < hmlen(calib->imu_factors); k++) {
//     imu_factor_t *factor = calib->imu_factors[k].value;
//     real_t **param_ptrs = factor->params;
//     int num_residuals = 15;
//     int num_params = 6;
//     int param_sizes[6] = {
//         7, // Pose i
//         3, // Vel i
//         6, // IMU biases i
//         7, // Pose j
//         3, // Vel j
//         6, // IMU biases j
//     };
//     ceres_problem_add_residual_block(problem,
//                                      &imu_factor_ceres_eval,
//                                      factor,
//                                      NULL,
//                                      NULL,
//                                      num_residuals,
//                                      num_params,
//                                      param_sizes,
//                                      param_ptrs);
//     num_factors++;
//     ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
//     ceres_set_parameterization(problem, param_ptrs[3], pose_pm);
//   }
//
//   {
//     camera_params_t *cam0_params = &calib->cam_params[0];
//     camera_params_t *cam1_params = &calib->cam_params[1];
//     ceres_set_parameter_constant(problem, cam0_params->data);
//     ceres_set_parameter_constant(problem, cam1_params->data);
//   }
//   {
//     extrinsic_t *cam0_ext = &calib->cam_exts[0];
//     extrinsic_t *cam1_ext = &calib->cam_exts[1];
//     ceres_set_parameter_constant(problem, cam0_ext->data);
//     ceres_set_parameter_constant(problem, cam1_ext->data);
//   }
//
//   {
//     time_delay_t *time_delay = calib->time_delay;
//     ceres_set_parameter_constant(problem, time_delay->data);
//   }
//
//   // Solve
//   // ceres_solve(problem, 100, 0);
//   ceres_solve(problem);
//   calib_imucam_print(calib);
//   // printf("num_factors: %d\n", num_factors);
//
//   // Clean up
//   timeline_free(timeline);
//   calib_imucam_free(calib);
//   ceres_free_problem(problem);
//
//   return 0;
// }

// int test_calib_gimbal_copy(void) {
//   const char *data_path = TEST_SIM_GIMBAL;
//   calib_gimbal_t *src = calib_gimbal_load(data_path);
//   calib_gimbal_t *dst = calib_gimbal_copy(src);
//
//   MU_ASSERT(src != NULL);
//   MU_ASSERT(dst != NULL);
//   MU_ASSERT(calib_gimbal_equals(src, dst));
//
//   // calib_gimbal_print(src);
//   calib_gimbal_free(src);
//   calib_gimbal_free(dst);
//
//   return 0;
// }
//
// int test_calib_gimbal_add_fiducial(void) {
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   real_t fiducial_pose[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
//   calib_gimbal_add_fiducial(calib, fiducial_pose);
//   MU_ASSERT(vec_equals(calib->fiducial_ext.data, fiducial_pose, 7));
//
//   calib_gimbal_free(calib);
//
//   return 0;
// }
//
// int test_calib_gimbal_add_pose(void) {
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   real_t pose[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
//   calib_gimbal_add_pose(calib, 0, pose);
//   MU_ASSERT(vec_equals(calib->poses[0].data, pose, 7));
//   MU_ASSERT(calib->num_poses == 1);
//
//   calib_gimbal_free(calib);
//
//   return 0;
// }
//
// int test_calib_gimbal_add_gimbal_extrinsic(void) {
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   real_t gimbal_ext[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
//   calib_gimbal_add_gimbal_extrinsic(calib, gimbal_ext);
//   MU_ASSERT(vec_equals(gimbal_ext, calib->gimbal_ext.data, 7));
//
//   calib_gimbal_free(calib);
//   return 0;
// }
//
// int test_calib_gimbal_add_gimbal_link(void) {
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   real_t link0[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
//   calib_gimbal_add_gimbal_link(calib, 0, link0);
//   MU_ASSERT(vec_equals(link0, calib->links[0].data, 7));
//   MU_ASSERT(calib->num_links == 1);
//
//   real_t link1[7] = {8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0};
//   calib_gimbal_add_gimbal_link(calib, 1, link1);
//   MU_ASSERT(vec_equals(link1, calib->links[1].data, 7));
//   MU_ASSERT(calib->num_links == 2);
//
//   calib_gimbal_free(calib);
//   return 0;
// }
//
// int test_calib_gimbal_add_camera(void) {
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   real_t cam0_params[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   real_t cam0_ext[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
//   real_t cam1_params[8] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
//   real_t cam1_ext[7] = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
//
//   calib_gimbal_add_camera(calib,
//                           0,
//                           cam_res,
//                           proj_model,
//                           dist_model,
//                           cam0_params,
//                           cam0_ext);
//   MU_ASSERT(vec_equals(cam0_params, calib->cam_params[0].data, 8));
//   MU_ASSERT(vec_equals(cam0_ext, calib->cam_exts[0].data, 7));
//   MU_ASSERT(calib->num_cams == 1);
//
//   calib_gimbal_add_camera(calib,
//                           1,
//                           cam_res,
//                           proj_model,
//                           dist_model,
//                           cam1_params,
//                           cam1_ext);
//   MU_ASSERT(vec_equals(cam1_params, calib->cam_params[1].data, 8));
//   MU_ASSERT(vec_equals(cam1_ext, calib->cam_exts[1].data, 7));
//   MU_ASSERT(calib->num_cams == 2);
//
//   calib_gimbal_free(calib);
//   return 0;
// }
//
// int test_calib_gimbal_add_remove_view(void) {
//   // Setup gimbal calibration
//   calib_gimbal_t *calib = calib_gimbal_malloc();
//
//   // -- Add fiducial
//   real_t fiducial[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   calib_gimbal_add_fiducial(calib, fiducial);
//
//   // -- Add pose
//   const timestamp_t ts = 0;
//   real_t pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   calib_gimbal_add_pose(calib, ts, pose);
//
//   // -- Add gimbal links
//   real_t link0[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   real_t link1[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   calib_gimbal_add_gimbal_link(calib, 0, link0);
//   calib_gimbal_add_gimbal_link(calib, 1, link1);
//
//   // -- Add camera
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   real_t cam0_params[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   real_t cam0_ext[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
//   calib_gimbal_add_camera(calib,
//                           0,
//                           cam_res,
//                           proj_model,
//                           dist_model,
//                           cam0_params,
//                           cam0_ext);
//
//   // -- Add view
//   const int pose_idx = 0;
//   const int view_idx = 0;
//   const int cam_idx = 0;
//   const int num_corners = 4;
//   const int tag_ids[4] = {0, 0, 0, 0};
//   const int corner_indices[4] = {0, 1, 2, 3};
//   const real_t object_points[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
//   const real_t keypoints[8] = {0, 1, 2, 3, 4, 5, 6, 7};
//   const real_t joints[3] = {0.0, 0.0, 0.0};
//   const int num_joints = 3;
//   calib_gimbal_add_view(calib,
//                         pose_idx,
//                         view_idx,
//                         ts,
//                         cam_idx,
//                         num_corners,
//                         tag_ids,
//                         corner_indices,
//                         object_points,
//                         keypoints,
//                         joints,
//                         num_joints);
//   MU_ASSERT(calib->num_cams == 1);
//   MU_ASSERT(calib->num_views == 1);
//   MU_ASSERT(calib->num_poses == 1);
//   MU_ASSERT(calib->num_links == 2);
//   MU_ASSERT(calib->num_joints == 3);
//   MU_ASSERT(calib->num_calib_factors == 4);
//   MU_ASSERT(calib->num_joint_factors == 3);
//   MU_ASSERT(calib->num_joints == 3);
//
//   // -- Remove view
//   calib_gimbal_remove_view(calib, view_idx);
//   MU_ASSERT(calib->num_cams == 1);
//   MU_ASSERT(calib->num_views == 0);
//   MU_ASSERT(calib->num_poses == 1);
//   MU_ASSERT(calib->num_links == 2);
//   MU_ASSERT(calib->num_joints == 3);
//   MU_ASSERT(calib->num_calib_factors == 0);
//   MU_ASSERT(calib->num_joint_factors == 0);
//   MU_ASSERT(calib->num_joints == 3);
//
//   // Clean up
//   calib_gimbal_free(calib);
//
//   return 0;
// }
//
// int test_calib_gimbal_load(void) {
//   const char *data_path = TEST_SIM_GIMBAL;
//   calib_gimbal_t *calib = calib_gimbal_load(data_path);
//   MU_ASSERT(calib != NULL);
//   calib_gimbal_print(calib);
//   calib_gimbal_free(calib);
//
//   return 0;
// }
//
// int test_calib_gimbal_save(void) {
//   const char *data_path = TEST_SIM_GIMBAL;
//   calib_gimbal_t *calib = calib_gimbal_load(data_path);
//   calib_gimbal_save(calib, "/tmp/estimates.yaml");
//   calib_gimbal_free(calib);
//
//   return 0;
// }
//
// static void compare_gimbal_calib(const calib_gimbal_t *gnd,
//                                  const calib_gimbal_t *est) {
//   assert(gnd->num_views == est->num_views);
//   assert(gnd->num_cams == est->num_cams);
//   assert(gnd->num_calib_factors == est->num_calib_factors);
//   assert(gnd->num_joint_factors == est->num_joint_factors);
//
//   // Compare estimated vs ground-truth
//   printf("\n");
//   {
//     printf("num_views: %d\n", gnd->num_views);
//     printf("num_cams: %d\n", gnd->num_cams);
//     printf("num_poses: %d\n", gnd->num_poses);
//     printf("num_links: %d\n", gnd->num_links);
//     printf("num_joints: %d\n", gnd->num_joints);
//     printf("num_calib_factors: %d\n", gnd->num_calib_factors);
//     printf("num_joint_factors: %d\n", gnd->num_joint_factors);
//
//     // Fiducial
//     {
//       real_t dr[3] = {0};
//       real_t dtheta = 0.0;
//       pose_diff2(gnd->fiducial_ext.data, est->fiducial_ext.data, dr, &dtheta);
//       printf("fiducial ");
//       printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
//       printf("dtheta: %f [deg]\n", rad2deg(dtheta));
//     }
//
//     // Links
//     for (int link_idx = 0; link_idx < est->num_links; link_idx++) {
//       real_t dr[3] = {0};
//       real_t dtheta = 0.0;
//       pose_diff2(gnd->links[link_idx].data,
//                  est->links[link_idx].data,
//                  dr,
//                  &dtheta);
//       printf("link_exts[%d] ", link_idx);
//       printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
//       printf("dtheta: %f [deg]\n", rad2deg(dtheta));
//     }
//
//     // Joints
//     real_t joints[3] = {0};
//     for (int view_idx = 0; view_idx < gnd->num_views; view_idx++) {
//       for (int joint_idx = 0; joint_idx < gnd->num_joints; joint_idx++) {
//         const real_t gnd_angle = gnd->joints[view_idx][joint_idx].data[0];
//         const real_t est_angle = est->joints[view_idx][joint_idx].data[0];
//         joints[joint_idx] += rad2deg(fabs(gnd_angle - est_angle));
//       }
//     }
//     for (int joint_idx = 0; joint_idx < gnd->num_joints; joint_idx++) {
//       printf("joint[%d] total diff: %f [deg]\n", joint_idx, joints[joint_idx]);
//     }
//
//     // Camera extrinsic
//     for (int cam_idx = 0; cam_idx < est->num_cams; cam_idx++) {
//       real_t dr[3] = {0};
//       real_t dtheta = 0.0;
//       pose_diff2(gnd->cam_exts[cam_idx].data,
//                  est->cam_exts[cam_idx].data,
//                  dr,
//                  &dtheta);
//       printf("cam_exts[%d] ", cam_idx);
//       printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
//       printf("dtheta: %f [deg]\n", rad2deg(dtheta));
//     }
//
//     // Camera parameters
//     for (int cam_idx = 0; cam_idx < est->num_cams; cam_idx++) {
//       real_t *cam_gnd = gnd->cam_params[cam_idx].data;
//       real_t *cam_est = est->cam_params[cam_idx].data;
//       real_t diff[8] = {0};
//       vec_sub(cam_gnd, cam_est, diff, 8);
//
//       printf("cam_params[%d] ", cam_idx);
//       print_vector("diff", diff, 8);
//     }
//   }
//   printf("\n");
// }
//
// int test_calib_gimbal_solve(void) {
//   // Setup
//   const int debug = 1;
//   // const char *data_path = TEST_SIM_GIMBAL;
//   // const char *data_path = "/tmp/calib_gimbal";
//   // const char *data_path = "/tmp/sim_gimbal";
//   const char *data_path = "/home/chutsu/calib_gimbal";
//   calib_gimbal_t *calib_gnd = calib_gimbal_load(data_path);
//   calib_gimbal_t *calib_est = calib_gimbal_load(data_path);
//   MU_ASSERT(calib_gnd != NULL);
//   MU_ASSERT(calib_est != NULL);
//
//   // Perturb parameters
//   // {
//   //   // printf("Ground Truth:\n");
//   //   // calib_gimbal_print(calib_gnd);
//   //   // printf("\n");
//
//   //   // Perturb
//   //   real_t dx[6] = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};
//   //   // pose_update(calib_est->fiducial_ext.data, dx);
//   //   // pose_update(calib_est->cam_exts[0].data, dx);
//   //   // pose_update(calib_est->cam_exts[1].data, dx);
//   //   for (int link_idx = 0; link_idx < calib_est->num_links; link_idx++) {
//   //     pose_update(calib_est->links[link_idx].data, dx);
//   //   }
//   //   // for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
//   //   //   for (int joint_idx = 0; joint_idx < calib_est->num_joints; joint_idx++) {
//   //   //     calib_est->joints[view_idx][joint_idx].data[0] += randf(-0.05, 0.05);
//   //   //   }
//   //   // }
//   //   // printf("\n");
//
//   //   //     printf("Initial:\n");
//   //   //     calib_gimbal_print(calib_est);
//   //   //     printf("\n");
//   // }
//   // if (debug) {
//   //   compare_gimbal_calib(calib_gnd, calib_est);
//   // }
//
//   // Solve
//   calib_gimbal_save(calib_est, "/tmp/estimates-before.yaml");
//   calib_gimbal_print(calib_est);
//   TIC(solve);
//   solver_t solver;
//   solver_setup(&solver);
//   solver.verbose = debug;
//   solver.max_iter = 10;
//   solver.param_order_func = &calib_gimbal_param_order;
//   solver.cost_func = &calib_gimbal_cost;
//   solver.linearize_func = &calib_gimbal_linearize_compact;
//   solver_solve(&solver, calib_est);
//   if (debug) {
//     calib_gimbal_print(calib_est);
//     compare_gimbal_calib(calib_gnd, calib_est);
//     PRINT_TOC("solve", solve);
//   }
//
//   // int sv_size = 0;
//   // int r_size = 0;
//   // param_order_t *hash = calib_gimbal_param_order(calib_est, &sv_size, &r_size);
//
//   // const int J_rows = r_size;
//   // const int J_cols = sv_size;
//   // real_t *J = CALLOC(real_t, r_size * sv_size);
//   // real_t *g = CALLOC(real_t, r_size * 1);
//   // real_t *r = CALLOC(real_t, r_size * 1);
//   // calib_gimbal_linearize(calib_est, J_rows, J_cols, hash, J, g, r);
//   // mat_save("/tmp/J.csv", J, J_rows, J_cols);
//
//   // free(J);
//   // free(g);
//   // free(r);
//   // param_order_free(hash);
//
//   // printf("Estimated:\n");
//   // calib_gimbal_print(calib);
//   // printf("\n");
//
//   // Clean up
//   // calib_gimbal_save(calib_gnd, "/tmp/estimates-gnd.yaml");
//   // calib_gimbal_save(calib_est, "/tmp/estimates-after.yaml");
//   calib_gimbal_free(calib_gnd);
//   calib_gimbal_free(calib_est);
//
//   return 0;
// }

#ifdef USE_CERES
int test_calib_gimbal_ceres_solve(void) {
  // Setup simulation data
  const char *data_path = TEST_SIM_GIMBAL;
  calib_gimbal_t *calib_gnd = calib_gimbal_load(data_path);
  calib_gimbal_t *calib_est = calib_gimbal_load(data_path);
  MU_ASSERT(calib_gnd != NULL);
  MU_ASSERT(calib_est != NULL);

  // Perturb parameters
  {
    // printf("Ground Truth:\n");
    // calib_gimbal_print(calib_gnd);
    // printf("\n");

    // Perturb
    // real_t dx[6] = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};
    // pose_update(calib_est->fiducial_ext.data, dx);
    // pose_update(calib_est->cam_exts[0].data, dx);
    // pose_update(calib_est->cam_exts[1].data, dx);
    // for (int link_idx = 0; link_idx < 2; link_idx++) {
    //   pose_update(calib_est->links[link_idx].data, dx);
    // }
    for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
      for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
        calib_est->joints[view_idx][joint_idx].data[0] += randf(-0.1, 0.1);
      }
    }
    // printf("\n");

    //     printf("Initial:\n");
    //     calib_gimbal_print(calib_est);
    //     printf("\n");
  }

  // Setup ceres problem
  ceres_init();
  ceres_problem_t *problem = ceres_create_problem();
  ceres_local_parameterization_t *pose_pm =
      ceres_create_pose_local_parameterization();

  const int num_residuals = 2;
  const int num_params = 10;
  for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib_est->num_cams; cam_idx++) {
      calib_gimbal_view_t *view = calib_est->views[view_idx][cam_idx];
      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        real_t *param_ptrs[] = {calib_est->fiducial_ext.data,
                                calib_est->gimbal_ext.data,
                                calib_est->poses[0].data,
                                calib_est->links[0].data,
                                calib_est->links[1].data,
                                calib_est->joints[view_idx][0].data,
                                calib_est->joints[view_idx][1].data,
                                calib_est->joints[view_idx][2].data,
                                calib_est->cam_exts[cam_idx].data,
                                calib_est->cam_params[cam_idx].data};
        int param_sizes[10] = {
            7, // Fiducial extrinsic
            7, // Gimbal extrinscis
            7, // Pose
            7, // Link0
            7, // Link1
            1, // Joint0
            1, // Joint1
            1, // Joint2
            7, // Camera extrinsic
            8, // Camera Parameters
        };
        ceres_problem_add_residual_block(problem,
                                         &calib_gimbal_factor_ceres_eval,
                                         &view->calib_factors[factor_idx],
                                         NULL,
                                         NULL,
                                         num_residuals,
                                         num_params,
                                         param_sizes,
                                         param_ptrs);
      } // For each corners
    }   // For each cameras
  }     // For each views

  // for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][0].data);
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][1].data);
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][2].data);
  // }

  // ceres_set_parameter_constant(problem, calib_est->fiducial_ext.data);
  ceres_set_parameter_constant(problem, calib_est->gimbal_ext.data);
  // ceres_set_parameter_constant(problem, calib_est->links[0].data);
  // ceres_set_parameter_constant(problem, calib_est->links[1].data);
  ceres_set_parameter_constant(problem, calib_est->cam_exts[0].data);
  ceres_set_parameter_constant(problem, calib_est->cam_exts[1].data);
  ceres_set_parameter_constant(problem, calib_est->cam_params[0].data);
  ceres_set_parameter_constant(problem, calib_est->cam_params[1].data);

  for (int view_idx = 0; view_idx < calib_est->num_poses; view_idx++) {
    ceres_set_parameter_constant(problem, calib_est->poses[view_idx].data);
    ceres_set_parameterization(problem,
                               calib_est->poses[view_idx].data,
                               pose_pm);
  }
  ceres_set_parameterization(problem, calib_est->fiducial_ext.data, pose_pm);
  ceres_set_parameterization(problem, calib_est->gimbal_ext.data, pose_pm);
  ceres_set_parameterization(problem, calib_est->links[0].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->links[1].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->cam_exts[0].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->cam_exts[1].data, pose_pm);
  TIC(solve);
  // ceres_solve(problem, 10, 0);
  ceres_solve(problem);
  PRINT_TOC("solve", solve);

  // Compare ground-truth vs estimates
  compare_gimbal_calib(calib_gnd, calib_est);

  // Clean up
  ceres_free_problem(problem);
  calib_gimbal_free(calib_gnd);
  calib_gimbal_free(calib_est);

  return 0;
}
#endif // USE_CERES

void test_suite(void) {
  MU_ADD_TEST(test_camchain);
  MU_ADD_TEST(test_calib_camera_factor);
  MU_ADD_TEST(test_calib_imucam_factor);
  // MU_ADD_TEST(test_calib_camera_mono_batch);
  // MU_ADD_TEST(test_calib_camera_mono_ceres);
  // MU_ADD_TEST(test_calib_camera_mono_incremental);
  // MU_ADD_TEST(test_calib_camera_stereo_batch);
  // MU_ADD_TEST(test_calib_camera_stereo_ceres);
  // MU_ADD_TEST(test_calib_imucam_view);
  // MU_ADD_TEST(test_calib_imucam_add_imu);
  // MU_ADD_TEST(test_calib_imucam_add_camera);
  // MU_ADD_TEST(test_calib_imucam_add_imu_event);
  // MU_ADD_TEST(test_calib_imucam_add_fiducial_event);
  // MU_ADD_TEST(test_calib_imucam_update);
  // MU_ADD_TEST(test_calib_imucam_batch);
  // MU_ADD_TEST(test_calib_imucam_batch_ceres);
}
MU_RUN_TESTS(test_suite)
