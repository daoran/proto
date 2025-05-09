#include "../src/xyz.h"

////////////////////////////
// CAMERA-IMU CALIBRATION //
////////////////////////////

// typedef struct calib_imucam_view_t {
//   timestamp_t ts;
//   int view_idx;
//   int cam_idx;
//   int num_corners;
//
//   int *tag_ids;
//   int *corner_indices;
//   real_t *object_points;
//   real_t *keypoints;
//
//   calib_imucam_factor_t *cam_factors;
// } calib_imucam_view_t;
//
// typedef struct calib_imucam_viewset_t {
//   timestamp_t key;
//   calib_imucam_view_t **value;
// } calib_imucam_viewset_t;
//
// typedef struct imu_factor_hash_t {
//   int64_t key;
//   imu_factor_t *value;
// } imu_factor_hash_t;
//
// typedef struct calib_imucam_t {
//   // Settings
//   int fix_fiducial;
//   int fix_poses;
//   int fix_velocities;
//   int fix_biases;
//   int fix_cam_params;
//   int fix_cam_exts;
//   int fix_time_delay;
//   int verbose;
//   int max_iter;
//
//   // Flags
//   int imu_ok;
//   int cams_ok;
//   int state_initialized;
//
//   // Counters
//   int num_imus;
//   int num_cams;
//   int num_views;
//   int num_cam_factors;
//   int num_imu_factors;
//
//   // Variables
//   timestamp_t *timestamps;
//   pose_hash_t *poses;
//   velocity_hash_t *velocities;
//   imu_biases_hash_t *imu_biases;
//   fiducial_t *fiducial;
//   extrinsic_t *cam_exts;
//   camera_params_t *cam_params;
//   extrinsic_t *imu_ext;
//   time_delay_t *time_delay;
//
//   // Data
//   fiducial_buffer_t *fiducial_buffer;
//   imu_params_t imu_params;
//   imu_buffer_t imu_buf;
//
//   // Views
//   calib_imucam_viewset_t *view_sets;
//   imu_factor_hash_t *imu_factors;
// } calib_imucam_t;
//
// calib_imucam_view_t *calib_imucam_view_malloc(const timestamp_t ts,
//                                               const int view_idx,
//                                               const int cam_idx,
//                                               const int num_corners,
//                                               const int *tag_ids,
//                                               const int *corner_indices,
//                                               const real_t *object_points,
//                                               const real_t *keypoints,
//                                               fiducial_t *fiducial,
//                                               pose_t *imu_pose,
//                                               extrinsic_t *imu_ext,
//                                               extrinsic_t *cam_ext,
//                                               camera_params_t *cam_params,
//                                               time_delay_t *time_delay);
// void calib_imucam_view_free(calib_imucam_view_t *view);
//
// calib_imucam_t *calib_imucam_malloc(void); void calib_imucam_free(calib_imucam_t *calib);
// void calib_imucam_print(calib_imucam_t *calib);
//
// void calib_imucam_add_imu(calib_imucam_t *calib,
//                           const real_t imu_rate,
//                           const real_t sigma_aw,
//                           const real_t sigma_gw,
//                           const real_t sigma_a,
//                           const real_t sigma_g,
//                           const real_t g,
//                           const real_t *imu_ext);
// void calib_imucam_add_camera(calib_imucam_t *calib,
//                              const int cam_idx,
//                              const int cam_res[2],
//                              const char *proj_model,
//                              const char *dist_model,
//                              const real_t *cam_params,
//                              const real_t *cam_ext);
//
// void calib_imucam_add_imu_event(calib_imucam_t *calib,
//                                 const timestamp_t ts,
//                                 const real_t acc[3],
//                                 const real_t gyr[3]);
// void calib_imucam_add_fiducial_event(calib_imucam_t *calib,
//                                      const timestamp_t ts,
//                                      const int cam_idx,
//                                      const int num_corners,
//                                      const int *tag_ids,
//                                      const int *corner_indices,
//                                      const real_t *object_points,
//                                      const real_t *keypoints);
// void calib_imucam_marginalize(calib_imucam_t *calib);
// int calib_imucam_update(calib_imucam_t *calib);
// void calib_imucam_errors(calib_imucam_t *calib,
//                          real_t *reproj_rmse,
//                          real_t *reproj_mean,
//                          real_t *reproj_median);
// param_order_t *calib_imucam_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size);
// void calib_imucam_cost(const void *data, real_t *r);
// void calib_imucam_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r);
// void calib_imucam_save_estimates(calib_imucam_t *calib);
// void calib_imucam_solve(calib_imucam_t *calib);

///////////////////////////////
// CALIB IMU-CAM CALIBRATION //
///////////////////////////////

// /**
//  * Malloc imucam calibration view.
//  */
// calib_imucam_view_t *calib_imucam_view_malloc(const timestamp_t ts,
//                                               const int view_idx,
//                                               const int cam_idx,
//                                               const int num_corners,
//                                               const int *tag_ids,
//                                               const int *corner_indices,
//                                               const real_t *object_points,
//                                               const real_t *keypoints,
//                                               fiducial_t *fiducial,
//                                               pose_t *imu_pose,
//                                               extrinsic_t *imu_ext,
//                                               extrinsic_t *cam_ext,
//                                               camera_params_t *cam_params,
//                                               time_delay_t *time_delay) {
//   calib_imucam_view_t *view = MALLOC(calib_imucam_view_t, 1);
//
//   // Properties
//   view->ts = ts;
//   view->view_idx = view_idx;
//   view->cam_idx = cam_idx;
//   view->num_corners = num_corners;
//
//   // Measurements
//   if (num_corners) {
//     view->tag_ids = malloc(sizeof(int) * num_corners);
//     view->corner_indices = malloc(sizeof(int) * num_corners);
//     view->object_points = malloc(sizeof(real_t) * num_corners * 3);
//     view->keypoints = malloc(sizeof(real_t) * num_corners * 2);
//   }
//
//   // Factors
//   view->cam_factors = malloc(sizeof(calib_imucam_factor_t) * num_corners);
//   assert(view->tag_ids != NULL);
//   assert(view->corner_indices != NULL);
//   assert(view->object_points != NULL);
//   assert(view->keypoints != NULL);
//   assert(view->cam_factors != NULL);
//
//   for (int i = 0; i < num_corners; i++) {
//     view->tag_ids[i] = tag_ids[i];
//     view->corner_indices[i] = corner_indices[i];
//     view->object_points[i * 3] = object_points[i * 3];
//     view->object_points[i * 3 + 1] = object_points[i * 3 + 1];
//     view->object_points[i * 3 + 2] = object_points[i * 3 + 2];
//     view->keypoints[i * 2] = keypoints[i * 2];
//     view->keypoints[i * 2 + 1] = keypoints[i * 2 + 1];
//   }
//
//   const real_t var[2] = {1.0, 1.0};
//   for (int i = 0; i < view->num_corners; i++) {
//     const int tag_id = tag_ids[i];
//     const int corner_idx = corner_indices[i];
//     const real_t *p_FFi = &object_points[i * 3];
//     const real_t *z = &keypoints[i * 2];
//     const real_t v[2] = {0.0, 0.0};
//
//     view->tag_ids[i] = tag_id;
//     view->corner_indices[i] = corner_idx;
//     view->object_points[i * 3] = p_FFi[0];
//     view->object_points[i * 3 + 1] = p_FFi[1];
//     view->object_points[i * 3 + 2] = p_FFi[2];
//     view->keypoints[i * 2] = z[0];
//     view->keypoints[i * 2 + 1] = z[1];
//
//     calib_imucam_factor_setup(&view->cam_factors[i],
//                               fiducial,
//                               imu_pose,
//                               imu_ext,
//                               cam_ext,
//                               cam_params,
//                               time_delay,
//                               cam_idx,
//                               tag_id,
//                               corner_idx,
//                               p_FFi,
//                               z,
//                               v,
//                               var);
//   }
//
//   return view;
// }
//
// /**
//  * Free imucam calibration view.
//  */
// void calib_imucam_view_free(calib_imucam_view_t *view) {
//   if (view) {
//     free(view->tag_ids);
//     free(view->corner_indices);
//     free(view->object_points);
//     free(view->keypoints);
//     free(view->cam_factors);
//     free(view);
//   }
// }
//
// /**
//  * Malloc imu-cam calibration problem.
//  */
// calib_imucam_t *calib_imucam_malloc(void) {
//   calib_imucam_t *calib = malloc(sizeof(calib_imucam_t) * 1);
//
//   // Settings
//   calib->fix_fiducial = 0;
//   calib->fix_poses = 0;
//   calib->fix_velocities = 0;
//   calib->fix_biases = 0;
//   calib->fix_cam_params = 0;
//   calib->fix_cam_exts = 0;
//   calib->fix_time_delay = 1;
//   calib->verbose = 1;
//   calib->max_iter = 30;
//
//   // Flags
//   calib->imu_ok = 0;
//   calib->cams_ok = 0;
//   calib->state_initialized = 0;
//
//   // Counters
//   calib->num_imus = 0;
//   calib->num_cams = 0;
//   calib->num_views = 0;
//   calib->num_cam_factors = 0;
//   calib->num_imu_factors = 0;
//
//   // Variables
//   calib->timestamps = NULL;
//   calib->poses = NULL;
//   calib->velocities = NULL;
//   calib->imu_biases = NULL;
//   calib->fiducial = NULL;
//   calib->cam_exts = NULL;
//   calib->cam_params = NULL;
//   calib->imu_ext = NULL;
//   calib->time_delay = NULL;
//
//   // Buffers
//   calib->fiducial_buffer = fiducial_buffer_malloc();
//   imu_buffer_setup(&calib->imu_buf);
//
//   // Factors
//   calib->view_sets = NULL;
//   calib->imu_factors = NULL;
//   hmdefault(calib->view_sets, NULL);
//   hmdefault(calib->imu_factors, NULL);
//
//   return calib;
// }
//
// /**
//  * Free camera calibration problem
//  */
// void calib_imucam_free(calib_imucam_t *calib) {
//   // Fiducial buffer
//   fiducial_buffer_free(calib->fiducial_buffer);
//
//   // View sets
//   if (calib->num_views) {
//     for (int i = 0; i < arrlen(calib->timestamps); i++) {
//       const timestamp_t ts = calib->timestamps[i];
//       calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
//       for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//         calib_imucam_view_free(cam_views[cam_idx]);
//       }
//       free(cam_views);
//     }
//   }
//   hmfree(calib->view_sets);
//
//   // IMU factors
//   for (int i = 0; i < hmlen(calib->imu_factors); i++) {
//     free(calib->imu_factors[i].value);
//   }
//   hmfree(calib->imu_factors);
//
//   // Timestamps
//   arrfree(calib->timestamps);
//   // -- Poses
//   for (int k = 0; k < hmlen(calib->poses); k++) {
//     free(calib->poses[k].value);
//   }
//   hmfree(calib->poses);
//   // -- Velocities
//   for (int k = 0; k < hmlen(calib->velocities); k++) {
//     free(calib->velocities[k].value);
//   }
//   hmfree(calib->velocities);
//   // -- IMU biases
//   for (int k = 0; k < hmlen(calib->imu_biases); k++) {
//     free(calib->imu_biases[k].value);
//   }
//   hmfree(calib->imu_biases);
//   // -- Others
//   free(calib->fiducial);
//   free(calib->cam_exts);
//   free(calib->cam_params);
//   free(calib->imu_ext);
//   free(calib->time_delay);
//
//   free(calib);
// }
//
// /**
//  * Print imu-cam calibration problem
//  */
// void calib_imucam_print(calib_imucam_t *calib) {
//   real_t reproj_rmse = 0.0;
//   real_t reproj_mean = 0.0;
//   real_t reproj_median = 0.0;
//   if (calib->num_views) {
//     calib_imucam_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
//   }
//
//   printf("settings:\n");
//   printf("  fix_fiducial: %d\n", calib->fix_fiducial);
//   printf("  fix_poses: %d\n", calib->fix_poses);
//   printf("  fix_cam_exts: %d\n", calib->fix_cam_exts);
//   printf("  fix_cam_params: %d\n", calib->fix_cam_params);
//   printf("  fix_time_delay: %d\n", calib->fix_time_delay);
//   printf("\n");
//
//   printf("statistics:\n");
//   printf("  num_cams: %d\n", calib->num_cams);
//   printf("  num_views: %d\n", calib->num_views);
//   printf("\n");
//
//   printf("reproj_errors:\n");
//   printf("  rmse: %f\n", reproj_rmse);
//   printf("  mean: %f\n", reproj_mean);
//   printf("  median: %f\n", reproj_median);
//   printf("\n");
//
//   if (calib->time_delay) {
//     printf("time_delay: %.4e  # [s] (cam_ts = imu_ts + time_delay)\n",
//            calib->time_delay->data[0]);
//     printf("\n");
//   }
//
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     camera_params_t *cam = &calib->cam_params[cam_idx];
//     char param_str[100] = {0};
//     vec2str(cam->data, 8, param_str);
//
//     printf("cam%d:\n", cam_idx);
//     printf("  resolution: [%d, %d]\n", cam->resolution[0], cam->resolution[1]);
//     printf("  proj_model: %s\n", cam->proj_model);
//     printf("  dist_model: %s\n", cam->dist_model);
//     printf("  param: %s\n", param_str);
//     printf("\n");
//
//     if (cam_idx > 0) {
//       char tf_str[20] = {0};
//       sprintf(tf_str, "T_cam0_cam%d", cam_idx);
//
//       extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//       POSE2TF(cam_ext->data, T);
//       printf("%s:\n", tf_str);
//       printf("  rows: 4\n");
//       printf("  cols: 4\n");
//       printf("  data: [\n");
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
//       printf("  ]\n");
//     }
//   }
//   printf("\n");
//
//   if (calib->imu_ext) {
//     extrinsic_t *imu_ext = calib->imu_ext;
//     POSE2TF(imu_ext->data, T);
//     printf("T_imu0_cam0:\n");
//     printf("  rows: 4\n");
//     printf("  cols: 4\n");
//     printf("  data: [\n");
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
//     printf("  ]\n");
//   }
// }
//
// /**
//  * Add imu to imu-cam calibration problem.
//  */
// void calib_imucam_add_imu(calib_imucam_t *calib,
//                           const real_t imu_rate,
//                           const real_t sigma_aw,
//                           const real_t sigma_gw,
//                           const real_t sigma_a,
//                           const real_t sigma_g,
//                           const real_t g,
//                           const real_t *imu_ext) {
//   assert(calib != NULL);
//   assert(imu_rate > 0);
//   assert(sigma_aw > 0);
//   assert(sigma_gw > 0);
//   assert(sigma_a > 0);
//   assert(sigma_g > 0);
//   assert(g > 9.0);
//   assert(imu_ext);
//
//   if (calib->num_imus == 1) {
//     LOG_ERROR("Currently only supports 1 IMU!\n");
//     return;
//   }
//
//   // IMU parameters
//   calib->imu_params.imu_idx = 0;
//   calib->imu_params.rate = imu_rate;
//   calib->imu_params.sigma_aw = sigma_aw;
//   calib->imu_params.sigma_gw = sigma_gw;
//   calib->imu_params.sigma_a = sigma_a;
//   calib->imu_params.sigma_g = sigma_g;
//   calib->imu_params.g = g;
//
//   // IMU extrinsic
//   calib->imu_ext = malloc(sizeof(extrinsic_t) * 1);
//   extrinsic_setup(calib->imu_ext, imu_ext);
//
//   // Time delay
//   calib->time_delay = malloc(sizeof(time_delay_t) * 1);
//   time_delay_setup(calib->time_delay, 0.0);
//
//   // Update
//   calib->num_imus++;
// }
//
// /**
//  * Add camera to imu-cam calibration problem.
//  */
// void calib_imucam_add_camera(calib_imucam_t *calib,
//                              const int cam_idx,
//                              const int cam_res[2],
//                              const char *proj_model,
//                              const char *dist_model,
//                              const real_t *cam_params,
//                              const real_t *cam_ext) {
//   assert(calib != NULL);
//   assert(cam_idx <= calib->num_cams);
//   assert(cam_res != NULL);
//   assert(proj_model != NULL);
//   assert(dist_model != NULL);
//   assert(cam_params != NULL);
//   assert(cam_ext != NULL);
//
//   if (cam_idx > (calib->num_cams - 1)) {
//     const int new_size = calib->num_cams + 1;
//     calib->cam_params = REALLOC(calib->cam_params, camera_params_t, new_size);
//     calib->cam_exts = REALLOC(calib->cam_exts, extrinsic_t, new_size);
//   }
//
//   camera_params_setup(&calib->cam_params[cam_idx],
//                       cam_idx,
//                       cam_res,
//                       proj_model,
//                       dist_model,
//                       cam_params);
//   extrinsic_setup(&calib->cam_exts[cam_idx], cam_ext);
//
//   // Fix both camera intrinsics and extrinsics
//   calib->cam_params[cam_idx].fix = 1;
//   calib->cam_exts[cam_idx].fix = 1;
//
//   // Update book keeping
//   calib->num_cams++;
//   calib->cams_ok = 1;
// }
//
// /** Estimate relative pose between camera and fiducial target T_CiF **/
// static int calib_imucam_estimate_relative_pose(calib_imucam_t *calib,
//                                                int *cam_idx,
//                                                real_t T_CiF[4 * 4]) {
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     const fiducial_event_t *data = calib->fiducial_buffer->data[i];
//     const camera_params_t *cam = &calib->cam_params[data->cam_idx];
//     const int status = solvepnp_camera(cam,
//                                        data->keypoints,
//                                        data->object_points,
//                                        data->num_corners,
//                                        T_CiF);
//     if (status != 0) {
//       return status;
//     }
//
//     *cam_idx = data->cam_idx;
//     break;
//   }
//
//   return 0;
// }
//
// /** Initialize fiducial pose T_WF **/
// static void calib_imucam_initialize_fiducial(calib_imucam_t *calib,
//                                              const timestamp_t ts) {
//   // Estimate relative pose T_CiF
//   int cam_idx = 0;
//   real_t T_CiF[4 * 4] = {0};
//   int status = calib_imucam_estimate_relative_pose(calib, &cam_idx, T_CiF);
//   if (status != 0) {
//     FATAL("FAILED!\n");
//     return;
//   }
//
//   // Form fiducial pose: T_WF
//   const pose_t *pose = hmgets(calib->poses, ts).value;
//   const extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//   const extrinsic_t *imu_ext = calib->imu_ext;
//   TF(pose->data, T_WS);
//   TF(cam_ext->data, T_C0Ci);
//   TF(imu_ext->data, T_SC0);
//   TF_CHAIN(T_SCi, 2, T_SC0, T_C0Ci);
//   TF_CHAIN(T_WF, 3, T_WS, T_SCi, T_CiF);
//   TF_VECTOR(T_WF, fiducial_pose);
//
//   // Form fiducial
//   calib->fiducial = malloc(sizeof(fiducial_t) * 1);
//   fiducial_setup(calib->fiducial, fiducial_pose);
// }
//
// /** Add state. **/
// static void calib_imucam_add_state(calib_imucam_t *calib,
//                                    const timestamp_t ts) {
//   // Check timestamp does not already exists
//   if (hmgets(calib->poses, ts).value != NULL) {
//     return;
//   }
//
//   // Setup state-variables
//   real_t pose_k[7] = {0};
//   real_t vel_k[3] = {0};
//   real_t ba_k[3] = {0};
//   real_t bg_k[3] = {0};
//
//   if (calib->state_initialized == 0) {
//     // Initialize first pose
//     real_t r_WS[3] = {0};
//     real_t q_WS[4] = {0};
//     real_t T_WS[4 * 4] = {0};
//     imu_initial_attitude(&calib->imu_buf, q_WS);
//     tf_qr(q_WS, r_WS, T_WS);
//     tf_vector(T_WS, pose_k);
//     calib->state_initialized = 1;
//
//   } else {
//     // Estimate relative pose T_CiF
//     int cam_idx = 0;
//     real_t T_CiF[4 * 4] = {0};
//     int status = calib_imucam_estimate_relative_pose(calib, &cam_idx, T_CiF);
//     if (status != 0) {
//       printf("Failed to estimate relative pose!\n");
//       return;
//     }
//
//     // Form T_WS
//     const extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//     const extrinsic_t *imu_ext = calib->imu_ext;
//     const fiducial_t *fiducial = calib->fiducial;
//     TF(fiducial->data, T_WF);
//     TF(cam_ext->data, T_C0Ci);
//     TF(imu_ext->data, T_SC0);
//     TF_INV(T_SC0, T_C0S);
//     TF_INV(T_CiF, T_FCi);
//     TF_INV(T_C0Ci, T_CiC0);
//     TF_CHAIN(T_WS, 4, T_WF, T_FCi, T_CiC0, T_C0S);
//     tf_vector(T_WS, pose_k);
//
//     // Estimate v_WS
//     const int last_idx = arrlen(calib->timestamps) - 1;
//     const timestamp_t last_ts = calib->timestamps[last_idx];
//     const real_t *pose_km1 = hmgets(calib->poses, last_ts).value->data;
//     vel_k[0] = pose_k[0] - pose_km1[0];
//     vel_k[1] = pose_k[1] - pose_km1[1];
//     vel_k[2] = pose_k[2] - pose_km1[2];
//   }
//
//   // Add timestamp
//   arrput(calib->timestamps, ts);
//
//   // Add state
//   // -- Pose
//   pose_t *imu_pose = malloc(sizeof(pose_t) * 1);
//   pose_setup(imu_pose, ts, pose_k);
//   hmput(calib->poses, ts, imu_pose);
//   // -- Velocity
//   velocity_t *vel = malloc(sizeof(velocity_t) * 1);
//   velocity_setup(vel, ts, vel_k);
//   hmput(calib->velocities, ts, vel);
//   // -- IMU biases
//   imu_biases_t *imu_biases = malloc(sizeof(imu_biases_t) * 1);
//   imu_biases_setup(imu_biases, ts, ba_k, bg_k);
//   hmput(calib->imu_biases, ts, imu_biases);
//
//   // Initialize fiducial
//   if (calib->fiducial == NULL) {
//     calib_imucam_initialize_fiducial(calib, ts);
//   }
// }
//
// /**
//  * Add IMU event.
//  */
// void calib_imucam_add_imu_event(calib_imucam_t *calib,
//                                 const timestamp_t ts,
//                                 const real_t acc[3],
//                                 const real_t gyr[3]) {
//   assert(calib != NULL);
//   assert(ts > 0);
//   assert(acc != NULL);
//   assert(gyr != NULL);
//   assert(calib->num_imus > 0);
//
//   // printf("add imu event:      %ld, ", ts);
//   // printf("acc: (%f, %f, %f), ", acc[0], acc[1], acc[2]);
//   // printf("gyr: (%f, %f, %f)\n", gyr[0], gyr[1], gyr[2]);
//
//   imu_buffer_add(&calib->imu_buf, ts, acc, gyr);
//   calib->imu_ok = 1;
// }
//
// /**
//  * Add camera event.
//  */
// void calib_imucam_add_fiducial_event(calib_imucam_t *calib,
//                                      const timestamp_t ts,
//                                      const int cam_idx,
//                                      const int num_corners,
//                                      const int *tag_ids,
//                                      const int *corner_indices,
//                                      const real_t *object_points,
//                                      const real_t *keypoints) {
//   assert(calib != NULL);
//   assert(calib->cams_ok);
//   assert(ts > 0);
//   assert(cam_idx >= 0);
//
//   // Pre-check
//   if (num_corners == 0 || calib->imu_ok == 0) {
//     return;
//   }
//
//   // printf("add fiducial event: %ld\n", ts);
//
//   // Add to buffer
//   fiducial_buffer_add(calib->fiducial_buffer,
//                       ts,
//                       cam_idx,
//                       num_corners,
//                       tag_ids,
//                       corner_indices,
//                       object_points,
//                       keypoints);
// }

// // /**
// //  * Marginalize oldest state variables in IMU-camera calibration.
// //  */
// // void calib_imucam_marginalize(calib_imucam_t *calib) {
// //   // // Setup marginalization factor
// //   // marg_factor_t *marg = marg_factor_malloc();
//
// //   // // Get first timestamp
// //   // const timestamp_t ts = calib->timestamps[0];
//
// //   // // Mark the pose at timestamp to be marginalized
// //   // pose_t *pose = hmgets(calib->poses, ts).value;
// //   // velocity_t *vel = hmgets(calib->velocities, ts).value;
// //   // imu_biases_t *biases = hmgets(calib->biases, ts).value;
// //   // assert(pose != NULL);
// //   // assert(vel != NULL);
// //   // assert(biases != NULL);
// //   // pose->marginalize = 1;
// //   // vel->marginalize = 1;
// //   // biases->marginalize = 1;
//
// //   // // Add calib camera factors to marginalization factor
// //   // calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
// //   // for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
// //   //   calib_imucam_view_t *view = cam_views[cam_idx];
// //   //   if (view == NULL) {
// //   //     continue;
// //   //   }
//
// //   //   for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
// //   //     marg_factor_add(marg, CALIB_IMUCAM_FACTOR, &view->factors[factor_idx]);
// //   //     calib->num_cam_factors--;
// //   //   }
// //   // }
//
// //   // // Add imu factor to marginalization factor
// //   // imu_factor_t *imu_factor = hmgets(calib->imu_factors, ts).value;
// //   // marg_factor_add(marg, IMU_FACTOR, imu_factor);
// //   // calib->num_imu_factors--;
//
// //   // // Add previous marginalization factor to new marginalization factor
// //   // if (calib->marg) {
// //   //   marg_factor_add(marg, MARG_FACTOR, calib->marg);
// //   // }
//
// //   // // Marginalize
// //   // marg_factor_marginalize(marg);
// //   // if (calib->marg) {
// //   //   marg_factor_free(calib->marg);
// //   // }
// //   // calib->marg = marg;
//
// //   // // Remove viewset
// //   // for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
// //   //   calib_imucam_view_free(cam_views[cam_idx]);
// //   // }
// //   // free(cam_views);
// //   // hmdel(calib->view_sets, ts);
//
// //   // // Remove IMU factor
// //   // free(imu_factor);
// //   // hmdel(calib->imu_factors, ts);
//
// //   // // Remove timestamp
// //   // arrdel(calib->timestamps, 0);
//
// //   // // Update number of views
// //   // calib->num_views--;
// // }
//
// /** Check update conditions. **/
// static int calib_imucam_update_precheck(calib_imucam_t *calib) {
//   // Check fiducial buffers empty?
//   if (calib->fiducial_buffer->size == 0) {
//     return -1;
//   }
//
//   // Check imu buffer empty?
//   if (calib->imu_buf.size == 0) {
//     return -1;
//   }
//
//   // Check timestamps are same
//   timestamp_t ts = 0;
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     if (i == 0) {
//       ts = calib->fiducial_buffer->data[i]->ts;
//     }
//
//     if (ts != calib->fiducial_buffer->data[i]->ts) {
//       return -2;
//     }
//   }
//
//   // Check IMU timestamp is after fiducial data
//   if (ts > imu_buffer_last_ts(&calib->imu_buf)) {
//     return -3;
//   }
//
//   return 0;
// }
//
// /*
// static real_t *calib_imucam_optflow(calib_imucam_t *calib,
//                                     const fiducial_event_t *fiducial) {
//   real_t *optflows = calloc(fiducial->num_corners * 2, sizeof(real_t));
//   return optflows;
//   // if (arrlen(calib->timestamps) < 2) {
//   //   return optflows;
//   // }
//
//   // const timestamp_t ts_km1 = calib->timestamps[arrlen(calib->timestamps) - 2];
//   // const timestamp_t ts_k = calib->timestamps[arrlen(calib->timestamps) - 1];
//   // const real_t dt = ts2sec(ts_k) - ts2sec(ts_km1);
//   // calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts_km1).value;
//   // if (cam_views == NULL || cam_views[fiducial->cam_idx] == NULL) {
//   //   return optflows;
//   // }
//
//   // const calib_imucam_view_t *prev_view = cam_views[fiducial->cam_idx];
//   // for (int i = 0; i < fiducial->num_corners; i++) {
//   //   // Get corner tag id, corner index and keypoint measurement
//   //   const int t_tag_id = fiducial->tag_ids[i];
//   //   const int t_corner_idx = fiducial->corner_indices[i];
//   //   const real_t *kp_k = &fiducial->keypoints[i * 2];
//
//   //   // Find same corner in previous view
//   //   int found_corner = 0;
//   //   real_t *kp_km1 = NULL;
//   //   for (int j = 0; j < prev_view->num_corners; j++) {
//   //     const int q_tag_id = prev_view->tag_ids[j];
//   //     const int q_corner_idx = prev_view->corner_indices[j];
//
//   //     const int tag_id_ok = (t_tag_id == q_tag_id);
//   //     const int corner_idx_ok = (t_corner_idx == q_corner_idx);
//
//   //     if (tag_id_ok && corner_idx_ok) {
//   //       found_corner = 1;
//   //       kp_km1 = &prev_view->keypoints[j * 2];
//   //       break;
//   //     }
//   //   }
//
//   //   // Calculate optical flow
//   //   if (found_corner) {
//   //     optflows[2 * i + 0] = (kp_k[0] - kp_km1[0]) * dt;
//   //     optflows[2 * i + 1] = (kp_k[1] - kp_km1[1]) * dt;
//   //   } else {
//   //     optflows[2 * i + 0] = 0;
//   //     optflows[2 * i + 1] = 0;
//   //   }
//   // }
//
//   // return optflows;
// }
// */
//
// /**
//  * Update IMU-Camera calibration problem.
//  */
// int calib_imucam_update(calib_imucam_t *calib) {
//   assert(calib != NULL);
//
//   // Pre-check
//   if (calib_imucam_update_precheck(calib) != 0) {
//     return -1;
//   }
//
//   // Add state
//   const timestamp_t ts = imu_buffer_last_ts(&calib->imu_buf);
//   calib_imucam_add_state(calib, ts);
//
//   // Form new view
//   calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
//   if (cam_views == NULL) {
//     cam_views = calloc(calib->num_cams, sizeof(calib_camera_view_t **));
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       cam_views[cam_idx] = NULL;
//     }
//     hmput(calib->view_sets, ts, cam_views);
//     calib->num_views++;
//   }
//
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     // Fiducial data
//     const fiducial_event_t *data = calib->fiducial_buffer->data[i];
//     const int cam_idx = data->cam_idx;
//     const int view_idx = calib->num_views;
//     pose_t *imu_pose = hmgets(calib->poses, ts).value;
//
//     calib_imucam_view_t *view =
//         calib_imucam_view_malloc(ts,
//                                  view_idx,
//                                  data->cam_idx,
//                                  data->num_corners,
//                                  data->tag_ids,
//                                  data->corner_indices,
//                                  data->object_points,
//                                  data->keypoints,
//                                  calib->fiducial,
//                                  imu_pose,
//                                  calib->imu_ext,
//                                  &calib->cam_exts[data->cam_idx],
//                                  &calib->cam_params[data->cam_idx],
//                                  calib->time_delay);
//
//     cam_views[cam_idx] = view;
//     calib->num_cam_factors += data->num_corners;
//   }
//
//   // Add imu factor
//   if (calib->num_views >= 2) {
//     // Pose, velocity and biases at km1
//     const size_t idx_km1 = arrlen(calib->timestamps) - 2;
//     const timestamp_t ts_km1 = calib->timestamps[idx_km1];
//     pose_t *pose_km1 = hmgets(calib->poses, ts_km1).value;
//     velocity_t *vel_km1 = hmgets(calib->velocities, ts_km1).value;
//     imu_biases_t *imu_biases_km1 = hmgets(calib->imu_biases, ts_km1).value;
//
//     // Pose, velocity and biases at k
//     const size_t idx_k = arrlen(calib->timestamps) - 1;
//     const timestamp_t ts_k = calib->timestamps[idx_k];
//     pose_t *pose_k = hmgets(calib->poses, ts_k).value;
//     velocity_t *vel_k = hmgets(calib->velocities, ts_k).value;
//     imu_biases_t *imu_biases_k = hmgets(calib->imu_biases, ts_k).value;
//
//     // printf("ts_km1: %ld, ts_k: %ld\n", ts_km1, ts_k);
//
//     // Form IMU factor
//     imu_factor_t *imu_factor = malloc(sizeof(imu_factor_t) * 1);
//     imu_factor_setup(imu_factor,
//                      &calib->imu_params,
//                      &calib->imu_buf,
//                      pose_km1,
//                      vel_km1,
//                      imu_biases_km1,
//                      pose_k,
//                      vel_k,
//                      imu_biases_k);
//     hmput(calib->imu_factors, ts, imu_factor);
//     calib->num_imu_factors++;
//
//     // Clear IMU buffer
//     imu_buffer_clear(&calib->imu_buf);
//   }
//
//   // Clear buffers
//   fiducial_buffer_clear(calib->fiducial_buffer);
//
//   return 0;
// }
//
// /**
//  * IMU-camera calibration reprojection errors.
//  */
// void calib_imucam_errors(calib_imucam_t *calib,
//                          real_t *reproj_rmse,
//                          real_t *reproj_mean,
//                          real_t *reproj_median) {
//   // Setup
//   const int N = calib->num_cam_factors;
//   const int r_size = N * 2;
//   real_t *r = calloc(r_size, sizeof(real_t));
//
//   // Evaluate residuals
//   int r_idx = 0;
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
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each timestamp
//
//   // Calculate reprojection errors
//   real_t *errors = calloc(N, sizeof(real_t));
//   for (int i = 0; i < N; i++) {
//     const real_t x = r[i * 2 + 0];
//     const real_t y = r[i * 2 + 1];
//     errors[i] = sqrt(x * x + y * y);
//   }
//
//   // Calculate RMSE
//   real_t sum = 0.0;
//   real_t sse = 0.0;
//   for (int i = 0; i < N; i++) {
//     sum += errors[i];
//     sse += errors[i] * errors[i];
//   }
//   *reproj_rmse = sqrt(sse / N);
//   *reproj_mean = sum / N;
//   *reproj_median = median(errors, N);
//
//   // Clean up
//   free(errors);
//   free(r);
// }
//
// /**
//  * IMU-camera calibration parameter order.
//  */
// param_order_t *calib_imucam_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size) {
//   // Setup parameter order
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//   param_order_t *hash = NULL;
//   int col_idx = 0;
//
//   // -- Add poses
//   for (int i = 0; i < hmlen(calib->poses); i++) {
//     param_order_add_pose(&hash, calib->poses[i].value, &col_idx);
//   }
//
//   // -- Add velocities
//   for (int i = 0; i < hmlen(calib->velocities); i++) {
//     param_order_add_velocity(&hash, calib->velocities[i].value, &col_idx);
//   }
//
//   // -- Add biases
//   for (int i = 0; i < hmlen(calib->imu_biases); i++) {
//     param_order_add_imu_biases(&hash, calib->imu_biases[i].value, &col_idx);
//   }
//
//   // -- Add fiducial
//   param_order_add_fiducial(&hash, calib->fiducial, &col_idx);
//
//   // -- Add camera extrinsic
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_extrinsic(&hash, &calib->cam_exts[cam_idx], &col_idx);
//   }
//
//   // -- Add IMU-camera extrinsic
//   param_order_add_extrinsic(&hash, calib->imu_ext, &col_idx);
//
//   // -- Add camera parameters
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_camera(&hash, &calib->cam_params[cam_idx], &col_idx);
//   }
//
//   // -- Add time delay
//   param_order_add_time_delay(&hash, calib->time_delay, &col_idx);
//
//   // Set state-vector and residual size
//   *sv_size = col_idx;
//   *r_size = (calib->num_cam_factors * 2) + (calib->num_imu_factors * 15);
//   // if (calib->marg) {
//   //   *r_size += calib->marg->r_size;
//   // }
//
//   return hash;
// }
//
// /**
//  * Calculate IMU-camera calibration problem cost.
//  */
// void calib_imucam_cost(const void *data, real_t *r) {
//   // Evaluate factors
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//
//   // -- Evaluate vision factors
//   int r_idx = 0;
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // -- Evaluate imu factors
//   for (int k = 0; k < hmlen(calib->imu_factors); k++) {
//     imu_factor_t *factor = calib->imu_factors[k].value;
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//     r_idx += factor->r_size;
//   }
//
//   // -- Evaluate marginalization factor
//   // if (calib->marg) {
//   //   marg_factor_eval(calib->marg);
//   //   vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//   // }
// }
//
// /**
//  * Linearize IMU-camera calibration problem.
//  */
// void calib_imucam_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r) {
//   // Evaluate factors
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//   int r_idx = 0;
//
//   // -- Evaluate calib camera factors
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//
//         solver_fill_hessian(hash,
//                             factor->num_params,
//                             factor->params,
//                             factor->jacs,
//                             factor->r,
//                             factor->r_size,
//                             sv_size,
//                             H,
//                             g);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // -- Evaluate imu factors
//   for (int k = 0; k < hmlen(calib->imu_factors); k++) {
//     imu_factor_t *factor = calib->imu_factors[k].value;
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//
//     solver_fill_hessian(hash,
//                         factor->num_params,
//                         factor->params,
//                         factor->jacs,
//                         factor->r,
//                         factor->r_size,
//                         sv_size,
//                         H,
//                         g);
//     r_idx += factor->r_size;
//   }
//
//   // -- Evaluate marginalization factor
//   // if (calib->marg) {
//   //   marg_factor_eval(calib->marg);
//   //   vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//
//   //   solver_fill_hessian(hash,
//   //                       calib->marg->num_params,
//   //                       calib->marg->params,
//   //                       calib->marg->jacs,
//   //                       calib->marg->r,
//   //                       calib->marg->r_size,
//   //                       sv_size,
//   //                       H,
//   //                       g);
//   // }
// }
//
// void calib_imucam_save_estimates(calib_imucam_t *calib) {
//   FILE *data = fopen("/tmp/calib_imucam.dat", "w");
//
//   // Cameras
//   fprintf(data, "# Camera Parameters\n");
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     char params_str[1024] = {0};
//     vec2csv(calib->cam_params[cam_idx].data, 8, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Camera extrinsics
//   fprintf(data, "# Camera Extrinsics\n");
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     char params_str[100] = {0};
//     vec2csv(calib->cam_exts[cam_idx].data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Camera extrinsics
//   fprintf(data, "# Camera-IMU Extrinsic\n");
//   {
//     char params_str[100] = {0};
//     vec2csv(calib->imu_ext->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Fiducial
//   fprintf(data, "# Fiducial\n");
//   {
//     char params_str[100] = {0};
//     vec2csv(calib->fiducial->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Poses
//   fprintf(data, "# Poses\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const pose_t *pose = hmgets(calib->poses, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(pose->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Velocities
//   fprintf(data, "# Velocities\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const velocity_t *vel = hmgets(calib->velocities, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(vel->data, 3, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Biases
//   fprintf(data, "# Biases\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const imu_biases_t *vel = hmgets(calib->imu_biases, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(vel->data, 3, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   fclose(data);
// }
//
// /**
//  * Solve IMU-camera calibration problem.
//  */
// void calib_imucam_solve(calib_imucam_t *calib) {
//   assert(calib != NULL);
//
//   if (calib->num_views == 0) {
//     return;
//   }
//
//   solver_t solver;
//   solver_setup(&solver);
//   solver.verbose = calib->verbose;
//   solver.max_iter = calib->max_iter;
//   solver.cost_func = &calib_imucam_cost;
//   solver.param_order_func = &calib_imucam_param_order;
//   solver.linearize_func = &calib_imucam_linearize_compact;
//   // solver.linsolve_func = &calib_imucam_linsolve;
//   solver_solve(&solver, calib);
//
//   if (calib->verbose) {
//     calib_imucam_print(calib);
//   }
// }
