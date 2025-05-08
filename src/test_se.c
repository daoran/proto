#include "munit.h"
#include "xyz.h"
#include "xyz_se.h"
#include "xyz_gui.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"
#define TEST_SIM_GIMBAL TEST_DATA_PATH "sim_gimbal"
#define TEST_CAM_APRIL TEST_DATA_PATH "cam_april"
#define TEST_IMU_APRIL TEST_DATA_PATH "imu_april"

#ifdef USE_CERES

/**
 * This is the equivalent of a use-defined CostFunction in the C++ Ceres API.
 * This is passed as a callback to the Ceres C API, which internally converts
 * the callback into a CostFunction.
 */
static int ceres_exp_residual(void *user_data,
                              double **parameters,
                              double *residuals,
                              double **jacobians) {
  double *measurement = (double *) user_data;
  double x = measurement[0];
  double y = measurement[1];
  double m = parameters[0][0];
  double c = parameters[1][0];
  residuals[0] = y - exp(m * x + c);
  if (jacobians == NULL) {
    return 1;
  }
  if (jacobians[0] != NULL) {
    jacobians[0][0] = -x * exp(m * x + c); /* dr/dm */
  }
  if (jacobians[1] != NULL) {
    jacobians[1][0] = -exp(m * x + c); /* dr/dc */
  }
  return 1;
}

int test_ceres_example(void) {
  int num_observations = 67;
  double data[] = {
      0.000000e+00, 1.133898e+00, 7.500000e-02, 1.334902e+00, 1.500000e-01,
      1.213546e+00, 2.250000e-01, 1.252016e+00, 3.000000e-01, 1.392265e+00,
      3.750000e-01, 1.314458e+00, 4.500000e-01, 1.472541e+00, 5.250000e-01,
      1.536218e+00, 6.000000e-01, 1.355679e+00, 6.750000e-01, 1.463566e+00,
      7.500000e-01, 1.490201e+00, 8.250000e-01, 1.658699e+00, 9.000000e-01,
      1.067574e+00, 9.750000e-01, 1.464629e+00, 1.050000e+00, 1.402653e+00,
      1.125000e+00, 1.713141e+00, 1.200000e+00, 1.527021e+00, 1.275000e+00,
      1.702632e+00, 1.350000e+00, 1.423899e+00, 1.425000e+00, 1.543078e+00,
      1.500000e+00, 1.664015e+00, 1.575000e+00, 1.732484e+00, 1.650000e+00,
      1.543296e+00, 1.725000e+00, 1.959523e+00, 1.800000e+00, 1.685132e+00,
      1.875000e+00, 1.951791e+00, 1.950000e+00, 2.095346e+00, 2.025000e+00,
      2.361460e+00, 2.100000e+00, 2.169119e+00, 2.175000e+00, 2.061745e+00,
      2.250000e+00, 2.178641e+00, 2.325000e+00, 2.104346e+00, 2.400000e+00,
      2.584470e+00, 2.475000e+00, 1.914158e+00, 2.550000e+00, 2.368375e+00,
      2.625000e+00, 2.686125e+00, 2.700000e+00, 2.712395e+00, 2.775000e+00,
      2.499511e+00, 2.850000e+00, 2.558897e+00, 2.925000e+00, 2.309154e+00,
      3.000000e+00, 2.869503e+00, 3.075000e+00, 3.116645e+00, 3.150000e+00,
      3.094907e+00, 3.225000e+00, 2.471759e+00, 3.300000e+00, 3.017131e+00,
      3.375000e+00, 3.232381e+00, 3.450000e+00, 2.944596e+00, 3.525000e+00,
      3.385343e+00, 3.600000e+00, 3.199826e+00, 3.675000e+00, 3.423039e+00,
      3.750000e+00, 3.621552e+00, 3.825000e+00, 3.559255e+00, 3.900000e+00,
      3.530713e+00, 3.975000e+00, 3.561766e+00, 4.050000e+00, 3.544574e+00,
      4.125000e+00, 3.867945e+00, 4.200000e+00, 4.049776e+00, 4.275000e+00,
      3.885601e+00, 4.350000e+00, 4.110505e+00, 4.425000e+00, 4.345320e+00,
      4.500000e+00, 4.161241e+00, 4.575000e+00, 4.363407e+00, 4.650000e+00,
      4.161576e+00, 4.725000e+00, 4.619728e+00, 4.800000e+00, 4.737410e+00,
      4.875000e+00, 4.727863e+00, 4.950000e+00, 4.669206e+00,
  };

  /* Note: Typically it is better to compact m and c into one block,
   * but in this case use separate blocks to illustrate the use of
   * multiple parameter blocks. */
  double m = 0.0;
  double c = 0.0;
  double *parameter_pointers[] = {&m, &c};
  int parameter_sizes[2] = {1, 1};
  ceres_problem_t *problem;
  ceres_init();
  problem = ceres_create_problem();

  /* Add all the residuals. */
  for (int i = 0; i < num_observations; ++i) {
    ceres_problem_add_residual_block(problem,
                                     ceres_exp_residual,
                                     &data[2 * i],
                                     NULL,
                                     NULL,
                                     1,
                                     2,
                                     parameter_sizes,
                                     parameter_pointers);
  }

  // ceres_solve(problem, 10, 0);
  ceres_solve(problem);
  ceres_free_problem(problem);
  // printf("Initial m: 0.0, c: 0.0\n");
  // printf("Final m: %g, c: %g\n", m, c);

  return 0;
}

#endif // USE_CERES

int test_solver_setup(void) {
  solver_t solver;
  solver_setup(&solver);
  return 0;
}

typedef struct cam_view_t {
  pose_t pose;
  ba_factor_t factors[1000];
  int num_factors;
  camera_params_t *cam_params;
} cam_view_t;

int test_solver_eval(void) {
  // Load test data
  // const char *dir_path = TEST_SIM_DATA "/cam0";
  // sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);

  // // Camera parameters
  // camera_params_t cam;
  // const int cam_idx = 0;
  // const int cam_res[2] = {640, 480};
  // const char *proj_model = "pinhole";
  // const char *dist_model = "radtan4";
  // const real_t params[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  // camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, params);
  //
  // // Setup features
  // // -- Load features csv
  // int num_rows = 0;
  // int num_cols = 0;
  // char *features_csv = TEST_SIM_DATA "/features.csv";
  // real_t **features_data = csv_data(features_csv, &num_rows, &num_cols);
  // size_t *feature_ids = MALLOC(size_t, num_rows);
  // real_t *feature_xyzs = MALLOC(real_t, num_rows * 3);
  // for (int i = 0; i < num_rows; i++) {
  //   feature_ids[i] = features_data[i][0];
  //   feature_xyzs[i * 3 + 0] = features_data[i][1];
  //   feature_xyzs[i * 3 + 1] = features_data[i][2];
  //   feature_xyzs[i * 3 + 2] = features_data[i][3];
  //   free(features_data[i]);
  // }
  // free(features_data);
  // // -- Add features to container
  // features_t *features = features_malloc();
  // features_add_xyzs(features, feature_ids, feature_xyzs, num_rows);
  // free(feature_ids);
  // free(feature_xyzs);
  //
  // // Loop over simulated camera frames
  // const real_t var[2] = {1.0, 1.0};
  // cam_view_t *cam_views = MALLOC(cam_view_t, cam_data->num_frames);
  // for (int k = 0; k < cam_data->num_frames; k++) {
  //   // Camera frame
  //   const sim_camera_frame_t *frame = cam_data->frames[k];
  //
  //   // Pose
  //   pose_t *pose = &cam_views[k].pose;
  //   pose_setup(pose, frame->ts, &cam_data->poses[k]);
  //
  //   // Add factors
  //   cam_views[k].num_factors = frame->num_measurements;
  //   for (int i = 0; i < frame->num_measurements; i++) {
  //     const int feature_id = frame->feature_ids[i];
  //     const real_t *z = &frame->keypoints[i];
  //     feature_t *f = NULL;
  //     features_get_xyz(features, feature_id, &f);
  //
  //     // Factor
  //     ba_factor_t *factor = &cam_views[k].factors[i];
  //     ba_factor_setup(factor, pose, f, &cam, z, var);
  //   }
  // }

  // solver_t solver;
  // solver_setup(&solver);

  // Clean up
  // sim_camera_data_free(cam_data);
  // free(cam_views);
  // features_free(features);

  return 0;
}

// int test_visual_odometry_batch(void) {
//   // Simulate features
//   const real_t origin[3] = {0.0, 0.0, 0.0};
//   const real_t dim[3] = {5.0, 5.0, 5.0};
//   const int num_features = 1000;
//   real_t features[3 * 1000] = {0};
//   sim_create_features(origin, dim, num_features, features);
//
//   // Camera configuration
//   const int res[2] = {640, 480};
//   const real_t fov = 90.0;
//   const real_t fx = pinhole_focal(res[0], fov);
//   const real_t fy = pinhole_focal(res[0], fov);
//   const real_t cx = res[0] / 2.0;
//   const real_t cy = res[1] / 2.0;
//   const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   camera_params_t cam0_params;
//   camera_params_setup(&cam0_params, 0, res, pmodel, dmodel, cam_vec);
//
//   // IMU-Camera0 extrinsic
//   const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
//   const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
//   TF_ER(cam0_ext_ypr, cam0_ext_r, T_BC0);
//   TF_VECTOR(T_BC0, cam0_ext);
//   TF_INV(T_BC0, T_C0B);
//
//   // Simulate data
//   sim_circle_t conf;
//   sim_circle_defaults(&conf);
//   sim_camera_data_t *cam0_data = sim_camera_circle_trajectory(&conf,
//                                                               T_BC0,
//                                                               &cam0_params,
//                                                               features,
//                                                               num_features);
//   // Setup factor graph
//   // feature_hash_t *feature_params = NULL;
//
//   // fgraph_t *fg = fgraph_malloc();
//   // // -- Add features
//   // for (int feature_id = 0; feature_id < num_features; feature_id++) {
//   //   fgraph_add_feature(fg, feature_id, features + feature_id * 3, 0);
//   // }
//   // // -- Add camera
//   // const int cam0_id = fgraph_add_camera(fg, 0, res, pmodel, dmodel, cam_vec, 0);
//   // const int cam0_ext_id = fgraph_add_cam_ext(fg, 0, cam0_ext, 0);
//
//   // for (size_t k = 0; k < cam0_data->num_frames; k++) {
//   //   const sim_camera_frame_t *cam0_frame = cam0_data->frames[k];
//
//   //   // Add pose
//   //   const real_t *cam0_pose = &cam0_data->poses[k * 7];
//   //   TF(cam0_pose, T_WC0);
//   //   TF_CHAIN(T_WB, 2, T_WC0, T_C0B);
//   //   TF_VECTOR(T_WB, body_pose);
//   //   pose_random_perturb(body_pose, 0.1, 0.1);
//   //   const int pose_id = fgraph_add_pose(fg, k, body_pose, 0);
//
//   //   // Add camera factors
//   //   for (int i = 0; i < cam0_frame->num_measurements; i++) {
//   //     const int feature_id = cam0_frame->feature_ids[i];
//   //     const real_t *kp = cam0_frame->keypoints + i * 2;
//   //     const int param_ids[4] = {pose_id, cam0_ext_id, feature_id, cam0_id};
//   //     const real_t var[2] = {1.0, 1.0};
//   //     fgraph_add_camera_factor(fg, param_ids, kp, var);
//   //   }
//   // }
//
//   // // Solve
//   // solver_t solver;
//   // solver_setup(&solver);
//
//   // solver.verbose = 1;
//   // solver.max_iter = 5;
//   // solver.cost_func = &fgraph_cost;
//   // solver.param_order_func = &fgraph_param_order;
//   // solver.linearize_func = &fgraph_linearize_compact;
//   // solver_solve(&solver, fg);
//
//   // Clean up
//   sim_camera_data_free(cam0_data);
//
//   return 0;
// }
//
// int test_inertial_odometry_batch(void) {
//   // Setup test data
//   imu_test_data_t test_data;
//   setup_imu_test_data(&test_data, 1.0, 0.1);
//
//   // Inertial Odometry
//   const int num_partitions = test_data.num_measurements / 20.0;
//   const size_t N = test_data.num_measurements / (real_t) num_partitions;
//   inertial_odometry_t *odom = malloc(sizeof(inertial_odometry_t) * 1);
//   // -- IMU params
//   odom->imu_params.imu_idx = 0;
//   odom->imu_params.rate = 200.0;
//   odom->imu_params.sigma_a = 0.08;
//   odom->imu_params.sigma_g = 0.004;
//   odom->imu_params.sigma_aw = 0.00004;
//   odom->imu_params.sigma_gw = 2.0e-6;
//   odom->imu_params.g = 9.81;
//   // -- Variables
//   odom->num_factors = 0;
//   odom->factors = malloc(sizeof(imu_factor_t) * num_partitions);
//   odom->poses = malloc(sizeof(pose_t) * num_partitions + 1);
//   odom->vels = malloc(sizeof(velocity_t) * num_partitions + 1);
//   odom->biases = malloc(sizeof(imu_biases_t) * num_partitions + 1);
//
//   const timestamp_t ts_i = test_data.timestamps[0];
//   const real_t *v_i = test_data.velocities[0];
//   const real_t ba_i[3] = {0, 0, 0};
//   const real_t bg_i[3] = {0, 0, 0};
//   pose_setup(&odom->poses[0], ts_i, test_data.poses[0]);
//   velocity_setup(&odom->vels[0], ts_i, v_i);
//   imu_biases_setup(&odom->biases[0], ts_i, ba_i, bg_i);
//
//   for (int i = 1; i < num_partitions; i++) {
//     const int ks = i * N;
//     const int ke = MIN((i + 1) * N - 1, test_data.num_measurements - 1);
//
//     // Setup imu buffer
//     imu_buffer_t imu_buf;
//     imu_buffer_setup(&imu_buf);
//     for (size_t k = 0; k < N; k++) {
//       const timestamp_t ts = test_data.timestamps[ks + k];
//       const real_t *acc = test_data.imu_acc[ks + k];
//       const real_t *gyr = test_data.imu_gyr[ks + k];
//       imu_buffer_add(&imu_buf, ts, acc, gyr);
//     }
//
//     // Setup parameters
//     const timestamp_t ts_j = test_data.timestamps[ke];
//     const real_t *v_j = test_data.velocities[ke];
//     const real_t ba_j[3] = {0, 0, 0};
//     const real_t bg_j[3] = {0, 0, 0};
//     pose_setup(&odom->poses[i], ts_j, test_data.poses[ke]);
//     velocity_setup(&odom->vels[i], ts_j, v_j);
//     imu_biases_setup(&odom->biases[i], ts_j, ba_j, bg_j);
//
//     // Setup IMU factor
//     imu_factor_setup(&odom->factors[i - 1],
//                      &odom->imu_params,
//                      &imu_buf,
//                      &odom->poses[i - 1],
//                      &odom->vels[i - 1],
//                      &odom->biases[i - 1],
//                      &odom->poses[i],
//                      &odom->vels[i],
//                      &odom->biases[i]);
//     odom->num_factors++;
//   }
//
//   // Save ground truth
//   inertial_odometry_save(odom, "/tmp/imu_odom-gnd.csv");
//
//   // Perturb ground truth
//   for (int k = 0; k <= odom->num_factors; k++) {
//     odom->poses[k].data[0] += randf(-1.0, 1.0);
//     odom->poses[k].data[1] += randf(-1.0, 1.0);
//     odom->poses[k].data[2] += randf(-1.0, 1.0);
//     quat_perturb(odom->poses[k].data + 3, 0, randf(-1e-1, 1e-1));
//     quat_perturb(odom->poses[k].data + 3, 1, randf(-1e-1, 1e-1));
//     quat_perturb(odom->poses[k].data + 3, 2, randf(-1e-1, 1e-1));
//
//     odom->vels[k].data[0] += randf(-1.0, 1.0);
//     odom->vels[k].data[1] += randf(-1.0, 1.0);
//     odom->vels[k].data[2] += randf(-1.0, 1.0);
//   }
//   inertial_odometry_save(odom, "/tmp/imu_odom-init.csv");
//
//   // Solve
//   solver_t solver;
//   solver_setup(&solver);
//   solver.verbose = 1;
//   solver.param_order_func = &inertial_odometry_param_order;
//   solver.cost_func = &inertial_odometry_cost;
//   solver.linearize_func = &inertial_odometry_linearize_compact;
//   solver_solve(&solver, odom);
//   inertial_odometry_save(odom, "/tmp/imu_odom-est.csv");
//
//   // Clean up
//   inertial_odometry_free(odom);
//   free_imu_test_data(&test_data);
//
//   return 0;
// }

// int test_visual_inertial_odometry_batch(void) {
//   // Simulate features
//   const real_t origin[3] = {0.0, 0.0, 0.0};
//   const real_t dim[3] = {5.0, 5.0, 5.0};
//   const int num_features = 1000;
//   real_t features[3 * 1000] = {0};
//   sim_create_features(origin, dim, num_features, features);

//   // Camera configuration
//   const int res[2] = {640, 480};
//   const real_t fov = 90.0;
//   const real_t fx = pinhole_focal(res[0], fov);
//   const real_t fy = pinhole_focal(res[0], fov);
//   const real_t cx = res[0] / 2.0;
//   const real_t cy = res[1] / 2.0;
//   const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   camera_params_t cam0_params;
//   camera_params_setup(&cam0_params, 0, res, pmodel, dmodel, cam_vec);

//   // IMU configuration
//   imu_params_t imu_params;
//   imu_params.imu_idx = 0;
//   imu_params.rate = 200.0;
//   imu_params.sigma_a = 0.08;
//   imu_params.sigma_g = 0.004;
//   imu_params.sigma_aw = 0.00004;
//   imu_params.sigma_gw = 2.0e-6;
//   imu_params.g = 9.81;

//   // IMU-Camera0 extrinsic
//   const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
//   const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
//   TF_ER(cam0_ext_ypr, cam0_ext_r, T_BC0);
//   TF_VECTOR(T_BC0, cam0_ext);
//   TF_INV(T_BC0, T_C0B);

//   // Simulate data
//   sim_circle_t conf;
//   sim_circle_defaults(&conf);
//   sim_imu_data_t *imu_data = sim_imu_circle_trajectory(&conf);
//   sim_camera_data_t *cam0_data = sim_camera_circle_trajectory(&conf,
//                                                               T_BC0,
//                                                               &cam0_params,
//                                                               features,
//                                                               num_features);
//   // Setup factor graph
//   fgraph_t *fg = fgraph_malloc();
//   // -- Add features
//   for (int feature_id = 0; feature_id < num_features; feature_id++) {
//     fgraph_add_feature(fg, feature_id, features + feature_id * 3, 0);
//   }
//   // -- Add camera
//   const int cam0_id = fgraph_add_camera(fg, 0, res, pmodel, dmodel, cam_vec, 0);
//   const int cam0_ext_id = fgraph_add_cam_ext(fg, 0, cam0_ext, 0);

//   int initialized = 0;
//   int pose_i_id = -1;
//   int vel_i_id = -1;
//   int biases_i_id = -1;
//   // for (size_t k = 1; k < cam0_data->num_frames; k++) {
//   for (size_t k = 1; k < 20; k++) {
//     const int64_t ts_i = cam0_data->timestamps[k - 1];
//     const int64_t ts_j = cam0_data->timestamps[k];
//     const sim_camera_frame_t *frame_i = cam0_data->frames[k - 1];
//     const sim_camera_frame_t *frame_j = cam0_data->frames[k];

//     // Add pose
//     const real_t *cam_pose_i = &cam0_data->poses[(k - 1) * 7];
//     const real_t *cam_pose_j = &cam0_data->poses[(k + 0) * 7];

//     // Add camera factors at i
//     if (initialized == 0) {
//       // Add pose i
//       TF(cam_pose_i, T_WC0_i);
//       TF_CHAIN(T_WB_i, 2, T_WC0_i, T_C0B);
//       TF_VECTOR(T_WB_i, pose_i);
//       // pose_random_perturb(pose_i, 0.1, 0.1);
//       pose_i_id = fgraph_add_pose(fg, ts_i, pose_i, 0);

//       // Add speed and biases at i
//       const real_t *vel_i = &imu_data->velocities[(k - 1) * 3];
//       const real_t ba_i[3] = {0.0, 0.0, 0.0};
//       const real_t bg_i[3] = {0.0, 0.0, 0.0};
//       vel_i_id = fgraph_add_velocity(fg, ts_i, vel_i, 0);
//       biases_i_id = fgraph_add_imu_biases(fg, ts_i, ba_i, bg_i, 0);

//       // // Add camera factors at i
//       // for (int i = 0; i < frame_i->num_measurements; i++) {
//       //   const int feature_id = frame_i->feature_ids[i];
//       //   const real_t *z = &frame_i->keypoints[i * 2];
//       //   const int param_ids[4] = {pose_i_id, cam0_ext_id, feature_id, cam0_id};
//       //   const real_t var[2] = {1.0, 1.0};
//       //   fgraph_add_camera_factor(fg, param_ids, z, var);
//       // }

//       initialized = 1;
//     }

//     // Add camera factors at j
//     {
//       // Add pose j
//       TF(cam_pose_j, T_WC0_j);
//       TF_CHAIN(T_WB_j, 2, T_WC0_j, T_C0B);
//       TF_VECTOR(T_WB_j, pose_j);
//       // pose_random_perturb(pose_j, 0.1, 0.1);
//       const int pose_j_id = fgraph_add_pose(fg, ts_j, pose_j, 0);

//       // Add speed and biases at j
//       const real_t *vel_j = &imu_data->velocities[k * 3];
//       const real_t ba_j[3] = {0.0, 0.0, 0.0};
//       const real_t bg_j[3] = {0.0, 0.0, 0.0};
//       const int vel_j_id = fgraph_add_velocity(fg, ts_j, vel_j, 0);
//       const int biases_j_id = fgraph_add_imu_biases(fg, ts_j, ba_j, bg_j, 0);

//       // // Add camera factors at j
//       // for (int i = 0; i < frame_j->num_measurements; i++) {
//       //   const int feature_id = frame_j->feature_ids[i];
//       //   const real_t *z = &frame_j->keypoints[i * 2];
//       //   const int param_ids[4] = {pose_j_id, cam0_ext_id, feature_id, cam0_id};
//       //   const real_t var[2] = {1.0, 1.0};
//       //   fgraph_add_camera_factor(fg, param_ids, z, var);
//       // }

//       // Add imu factor between i and j
//       int param_ids[6] = {0};
//       param_ids[0] = pose_i_id;
//       param_ids[1] = vel_i_id;
//       param_ids[2] = biases_i_id;
//       param_ids[3] = pose_j_id;
//       param_ids[4] = vel_j_id;
//       param_ids[5] = biases_j_id;
//       imu_buffer_t imu_buf;
//       sim_imu_measurements(imu_data, ts_i, ts_j, &imu_buf);
//       fgraph_add_imu_factor(fg, 0, param_ids, &imu_params, &imu_buf);

//       // Update
//       pose_i_id = pose_j_id;
//       vel_i_id = vel_j_id;
//       biases_i_id = biases_j_id;
//     }
//   }

//   // Solve
//   solver_t solver;
//   solver_setup(&solver);

//   solver.verbose = 1;
//   solver.max_iter = 5;
//   solver.cost_func = &fgraph_cost;
//   solver.param_order_func = &fgraph_param_order;
//   solver.linearize_func = &fgraph_linearize_compact;
//   solver_solve(&solver, fg);

//   // Clean up
//   sim_imu_data_free(imu_data);
//   sim_camera_data_free(cam0_data);
//   fgraph_free(fg);

//   return 0;
// }

// void vis_kitti_scan(gl_points3d_t *gl_points,
//                     const gl_float_t *pcd,
//                     const size_t num_points) {
//   gl_float_t point_size = 2.0;
//   gl_float_t *points_data = malloc(sizeof(gl_float_t) * num_points * 6);
//
//   for (size_t i = 0; i < num_points; ++i) {
//     // Point positions
//     points_data[i * 6 + 0] = pcd[i * 3 + 1];
//     points_data[i * 6 + 1] = pcd[i * 3 + 2] + 1.6;
//     points_data[i * 6 + 2] = -pcd[i * 3 + 0];
//
//     // Point color
//     gl_float_t r = 0.0f;
//     gl_float_t g = 0.0f;
//     gl_float_t b = 0.0f;
//     gl_jet_colormap((pcd[i * 3 + 2] + 1.6) / 3.0, &r, &g, &b);
//     points_data[i * 6 + 3] = r;
//     points_data[i * 6 + 4] = g;
//     points_data[i * 6 + 5] = b;
//   }
//   gl_points3d_update(gl_points, points_data, num_points, point_size);
//   free(points_data);
// }
//
// float *kitti_lidar_points(const char *pcd_path, size_t *n) {
//   // Load Kitti LIDAR points [x, y, z, intensity]
//   size_t num_points = 0;
//   float *raw_points = kitti_load_points(pcd_path, &num_points);
//
//   // Extract only the relevant parts
//   float *points_xyz = malloc(sizeof(float) * 3 * num_points);
//   for (size_t i = 0; i < num_points; ++i) {
//     points_xyz[i * 3 + 0] = raw_points[i * 4 + 0];
//     points_xyz[i * 3 + 1] = raw_points[i * 4 + 1];
//     points_xyz[i * 3 + 2] = raw_points[i * 4 + 2];
//   }
//
//   // Downsample with octree
//   const float voxel_size = 0.5;
//   const size_t voxel_limit = 10;
//   float *points_downsampled =
//       octree_downsample(points_xyz, num_points, voxel_size, voxel_limit, n);
//
//   // Clean up
//   free(raw_points);
//   free(points_xyz);
//
//   return points_downsampled;
// }
//
// void icp_jacobian(const float pose_est[4 * 4], float J[3 * 6]) {
//   const float neye[3 * 3] = {-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
//   TF_ROT(pose_est, R_est);
//
//   // J = zeros((3, 6))
//   // J[0:3, 0:3] = -1.0 * eye(3)
//   // J[0:3, 3:6] = R_est @ hat(p_dst_est[:, i])
//   mat_block_set(J, 6, 0, 2, 0, 2, neye);
// }
//
// void estimate_pose(const pcd_t *pcd_km1,
//                    const pcd_t *pcd_k,
//                    const float T_WB_km1[4 * 4],
//                    float T_WB_k[4 * 4]) {
//   assert(pcd_km1);
//   assert(pcd_k);
//   assert(T_WB_km1);
//   assert(T_WB_k);
//
//   // Setup
//   const float dist_threshold = 0.1;
//   const kdtree_t *kdtree = kdtree_malloc(pcd_km1->data, pcd_km1->num_points);
//   const size_t n = MIN(pcd_km1->num_points, pcd_k->num_points);
//   float *points_i = malloc(sizeof(float) * 3 * n);
//   float *points_j = malloc(sizeof(float) * 3 * n);
//   float *r = malloc(sizeof(float) * 3 * n);
//
//   // Find correspondences
//   size_t m = 0;
//   for (size_t j = 0; j < pcd_k->num_points; ++j) {
//     float best_point[3] = {0};
//     float best_dist = INFINITY;
//     kdtree_nn(kdtree, &pcd_k->data[j * 3], best_point, &best_dist);
//     if (best_dist < dist_threshold) {
//       points_i[m * 3 + 0] = best_point[0];
//       points_i[m * 3 + 1] = best_point[1];
//       points_i[m * 3 + 2] = best_point[2];
//
//       points_j[m * 3 + 0] = pcd_k->data[j * 3 + 0];
//       points_j[m * 3 + 1] = pcd_k->data[j * 3 + 1];
//       points_j[m * 3 + 2] = pcd_k->data[j * 3 + 2];
//       m++;
//     }
//   }
//
//   float *H = malloc(sizeof(float) * 6 * 6);
//   for (size_t idx = 0; idx < m; ++idx) {
//     r[idx * 3 + 0] = points_i[idx * 3 + 0] - points_j[idx * 3 + 0];
//     r[idx * 3 + 1] = points_i[idx * 3 + 1] - points_j[idx * 3 + 1];
//     r[idx * 3 + 2] = points_i[idx * 3 + 2] - points_j[idx * 3 + 2];
//
//     // Calculate jacobian
//
//     // Form hessian
//   }
//
//   // perform least squares
//
//   // update pose
//
//   return;
// }
//
// int test_icp(void) {
//   // Setup
//   const char *window_title = "viz";
//   const int window_width = 1024;
//   const int window_height = 768;
//   gui_t *gui = gui_malloc(window_title, window_width, window_height);
//
//   // Grid
//   gl_float_t grid_size = 0.5f;
//   gl_float_t grid_lw = 5.0f;
//   gl_color_t grid_color = (gl_color_t){0.9, 0.4, 0.2};
//   gl_grid3d_t *gl_grid = gl_grid3d_malloc(grid_size, grid_color, grid_lw);
//
//   // KITTI
//   const char *data_dir = "/data/kitti_raw/2011_09_26";
//   const char *seq_name = "2011_09_26_drive_0001_sync";
//   kitti_raw_t *kitti = kitti_raw_load(data_dir, seq_name);
//   gl_points3d_t *gl_points = gl_points3d_malloc(NULL, 0, 0);
//
//   int pcd_index = 0;
//   double time_prev = glfwGetTime();
//   pcd_t *pcd_km1 = NULL;
//   pcd_t *pcd_k = NULL;
//   float T_WB_km1[4 * 4] = {0};
//   float T_WB_k[4 * 4] = {0};
//
//   while (gui_poll(gui)) {
//     double time_dt = glfwGetTime() - time_prev;
//     if (pcd_index >= kitti->velodyne->num_timestamps) {
//       break;
//     }
//
//     if (*gui->key_n || time_dt > 0.1) {
//       printf("[pcd_index]: %d\n", pcd_index);
//
//       // Load lidar points
//       const timestamp_t ts_start = kitti->velodyne->timestamps_start[pcd_index];
//       const timestamp_t ts_end = kitti->velodyne->timestamps_end[pcd_index];
//       const char *pcd_path = kitti->velodyne->pcd_paths[pcd_index];
//       size_t num_points = 0;
//       float *points = kitti_lidar_points(pcd_path, &num_points);
//       float *time_diffs = malloc(sizeof(float) * num_points);
//       for (int i = 0; i < num_points; ++i) {
//         time_diffs[i] = 0.0;
//       }
//       pcd_k = pcd_malloc(ts_start, ts_end, points, time_diffs, num_points);
//
//       // Estimate
//       estimate_pose(pcd_km1, pcd_k, T_WB_km1, T_WB_k);
//       // -- Update point cloud
//       pcd_free(pcd_km1);
//       pcd_km1 = pcd_k;
//       pcd_k = NULL;
//
//       // Visualize
//       vis_kitti_scan(gl_points, points, num_points);
//       time_prev = glfwGetTime();
//
//       // Clean up
//       free(points);
//       free(time_diffs);
//
//       // Update
//       pcd_index++;
//     }
//
//     draw_grid3d(gl_grid);
//     draw_points3d(gl_points);
//     gui_update(gui);
//   }
//
//   // Clean up
//   pcd_free(pcd_km1);
//   pcd_free(pcd_k);
//   kitti_raw_free(kitti);
//   gl_points3d_free(gl_points);
//   gl_grid3d_free(gl_grid);
//   gui_free(gui);
//
//   return 0;
// }

int test_assoc_pose_data(void) {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "./test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "./test_data/euroc/MH01_estimate.csv";

  // Load ground-truth poses
  int num_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &num_gnd_poses);
  printf("num_gnd_poses: %d\n", num_gnd_poses);

  // Load estimate poses
  int num_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &num_est_poses);
  printf("num_est_poses: %d\n", num_est_poses);

  // Associate data
  size_t num_matches = 0;
  int **matches = assoc_pose_data(gnd_poses,
                                  num_gnd_poses,
                                  est_poses,
                                  num_est_poses,
                                  threshold,
                                  &num_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("num_matches: %ld\n", num_matches);

  // Save matches to file
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < num_matches; i++) {
    uint64_t gnd_ts = gnd_poses[matches[i][0]].ts;
    uint64_t est_ts = est_poses[matches[i][1]].ts;
    double t_diff = fabs(ts2sec(gnd_ts - est_ts));
    if (t_diff > threshold) {
      printf("ERROR! Time difference > threshold!\n");
      printf("ground_truth_index: %d\n", matches[i][0]);
      printf("estimate_index: %d\n", matches[i][1]);
      break;
    }
    fprintf(matches_csv, "%d,%d\n", matches[i][0], matches[i][1]);
  }
  fclose(matches_csv);

  // Clean up
  for (size_t i = 0; i < num_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

void test_suite(void) {
#ifdef USE_CERES
  MU_ADD_TEST(test_ceres_example);
#endif // USE_CERES
  MU_ADD_TEST(test_solver_setup);
  // MU_ADD_TEST(test_solver_eval);
  // MU_ADD_TEST(test_visual_odometry_batch);
  // MU_ADD_TEST(test_inertial_odometry_batch);
  // MU_ADD_TEST(test_visual_inertial_odometry_batch);
  // MU_ADD_TEST(test_icp);
  // MU_ADD_TEST(test_assoc_pose_data);
}
MU_RUN_TESTS(test_suite)
