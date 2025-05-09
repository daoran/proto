#include "../src/xyz.h"

/******************************************************************************
 * CALIB_FRAME
 *****************************************************************************/

typedef struct calib_frame_t {
  timestamp_t ts;
  int view_idx;
  int cam_idx;
  int num_corners;

  int *tag_ids;
  int *corner_indices;
  real_t *object_points;
  real_t *keypoints;

  struct calib_camera_factor_t *factors;
} calib_frame_t;

/**
 * Malloc camera calibration view.
 */
calib_frame_t *calib_frame_malloc(const timestamp_t ts,
                                  const int view_idx,
                                  const int cam_idx,
                                  const int num_corners,
                                  const int *tag_ids,
                                  const int *corner_indices,
                                  const real_t *object_points,
                                  const real_t *keypoints,
                                  pose_t *pose,
                                  extrinsic_t *cam_ext,
                                  camera_params_t *cam_params) {
  calib_frame_t *view = malloc(sizeof(calib_frame_t) * 1);

  // Properties
  view->ts = ts;
  view->view_idx = view_idx;
  view->cam_idx = cam_idx;
  view->num_corners = num_corners;

  // Measurements
  if (num_corners) {
    view->tag_ids = malloc(sizeof(int) * num_corners);
    view->corner_indices = malloc(sizeof(int) * num_corners);
    view->object_points = malloc(sizeof(real_t) * num_corners * 3);
    view->keypoints = malloc(sizeof(real_t) * num_corners * 2);
    assert(view->tag_ids != NULL);
    assert(view->corner_indices != NULL);
    assert(view->object_points != NULL);
    assert(view->keypoints != NULL);
  }

  // Factors
  view->factors = malloc(sizeof(struct calib_camera_factor_t) * num_corners);
  assert(view->factors != NULL);

  for (int i = 0; i < num_corners; i++) {
    view->tag_ids[i] = tag_ids[i];
    view->corner_indices[i] = corner_indices[i];
    view->object_points[i * 3] = object_points[i * 3];
    view->object_points[i * 3 + 1] = object_points[i * 3 + 1];
    view->object_points[i * 3 + 2] = object_points[i * 3 + 2];
    view->keypoints[i * 2] = keypoints[i * 2];
    view->keypoints[i * 2 + 1] = keypoints[i * 2 + 1];
  }

  const real_t var[2] = {1.0, 1.0};
  for (int i = 0; i < view->num_corners; i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indices[i];
    const real_t *p_FFi = &object_points[i * 3];
    const real_t *z = &keypoints[i * 2];

    view->tag_ids[i] = tag_id;
    view->corner_indices[i] = corner_idx;
    view->object_points[i * 3] = p_FFi[0];
    view->object_points[i * 3 + 1] = p_FFi[1];
    view->object_points[i * 3 + 2] = p_FFi[2];
    view->keypoints[i * 2] = z[0];
    view->keypoints[i * 2 + 1] = z[1];

    calib_camera_factor_setup(&view->factors[i],
                              pose,
                              cam_ext,
                              cam_params,
                              cam_idx,
                              tag_id,
                              corner_idx,
                              p_FFi,
                              z,
                              var);
  }

  return view;
}

/**
 * Free camera calibration view.
 */
void calib_frame_free(calib_frame_t *view) {
  if (view) {
    free(view->tag_ids);
    free(view->corner_indices);
    free(view->object_points);
    free(view->keypoints);
    free(view->factors);
    free(view);
  }
}

/******************************************************************************
 * CAMERA CALIBRATOR
 *****************************************************************************/

typedef struct calib_frameset_t {
  timestamp_t key;
  calib_frame_t **value;
} calib_frameset_t;

typedef struct calib_camera_t {
  // Settings
  int fix_cam_params;
  int fix_cam_exts;
  int verbose;
  int max_iter;

  // Flags
  int cams_ok;

  // Counters
  int num_cams;
  int num_views;
  int num_factors;

  // Variables
  timestamp_t *timestamps;
  // pose_hash_t *poses;
  extrinsic_t *cam_exts;
  camera_params_t *cam_params;

  // Factors
  calib_frameset_t *view_sets;
  marg_factor_t *marg;
} calib_camera_t;

/**
 * Malloc camera calibration problem
 */
calib_camera_t *calib_camera_malloc(void) {
  calib_camera_t *calib = malloc(sizeof(calib_camera_t) * 1);

  // Settings
  calib->fix_cam_exts = 0;
  calib->fix_cam_params = 0;
  calib->verbose = 1;
  calib->max_iter = 20;

  // Flags
  calib->cams_ok = 0;

  // Counters
  calib->num_cams = 0;
  calib->num_views = 0;
  calib->num_factors = 0;

  // Variables
  calib->timestamps = NULL;
  // calib->poses = NULL;
  calib->cam_exts = NULL;
  calib->cam_params = NULL;
  // hmdefault(calib->poses, NULL);

  // Factors
  calib->view_sets = NULL;
  // hmdefault(calib->view_sets, NULL);
  calib->marg = NULL;

  return calib;
}

/**
 * Free camera calibration problem
 */
void calib_camera_free(calib_camera_t *calib) {
  free(calib->cam_exts);
  free(calib->cam_params);

  // if (calib->num_views) {
  //   // View sets
  //   for (int i = 0; i < arrlen(calib->timestamps); i++) {
  //     const timestamp_t ts = calib->timestamps[i];
  //     calib_frame_t **cam_views = hmgets(calib->view_sets, ts).value;
  //     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
  //       calib_frame_free(cam_views[cam_idx]);
  //     }
  //     free(cam_views);
  //   }
  //
  //   // Timestamps
  //   arrfree(calib->timestamps);
  //
  //   // Poses
  //   for (int i = 0; i < hmlen(calib->poses); i++) {
  //     free(calib->poses[i].value);
  //   }
  // }
  // hmfree(calib->poses);
  // hmfree(calib->view_sets);

  // Free previous marg_factor_t
  marg_factor_free(calib->marg);

  free(calib);
}

// /**
//  * Camera calibration reprojection errors.
//  */
// void calib_camera_errors(calib_camera_t *calib,
//                          real_t *reproj_rmse,
//                          real_t *reproj_mean,
//                          real_t *reproj_median) {
//   // Setup
//   const int N = calib->num_factors;
//   const int r_size = N * 2;
//   real_t *r = calloc(r_size, sizeof(real_t));
//
//   // Evaluate residuals
//   int r_idx = 0;
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_frame_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         struct calib_camera_factor_t *factor = &view->factors[factor_idx];
//         calib_camera_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
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

/**
 * Print camera calibration.
 */
void calib_camera_print(calib_camera_t *calib) {
  // real_t reproj_rmse = 0.0;
  // real_t reproj_mean = 0.0;
  // real_t reproj_median = 0.0;
  // calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);

  printf("settings:\n");
  printf("  fix_cam_exts: %d\n", calib->fix_cam_exts);
  printf("  fix_cam_params: %d\n", calib->fix_cam_params);
  printf("\n");

  printf("statistics:\n");
  printf("  num_cams: %d\n", calib->num_cams);
  printf("  num_views: %d\n", calib->num_views);
  printf("  num_factors: %d\n", calib->num_factors);
  printf("\n");

  // printf("reproj_errors:\n");
  // printf("  rmse:   %f  # [px]\n", reproj_rmse);
  // printf("  mean:   %f  # [px]\n", reproj_mean);
  // printf("  median: %f  # [px]\n", reproj_median);
  // printf("\n");

  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    camera_params_t *cam = &calib->cam_params[cam_idx];
    char param_str[100] = {0};
    vec2str(cam->data, 8, param_str);

    printf("cam%d:\n", cam_idx);
    printf("  resolution: [%d, %d]\n", cam->resolution[0], cam->resolution[1]);
    printf("  proj_model: %s\n", cam->proj_model);
    printf("  dist_model: %s\n", cam->dist_model);
    printf("  param: %s\n", param_str);
    printf("\n");

    if (cam_idx > 0) {
      char tf_str[20] = {0};
      sprintf(tf_str, "T_cam0_cam%d", cam_idx);

      POSE2TF(calib->cam_exts[cam_idx].data, T);
      printf("%s:\n", tf_str);
      printf("  rows: 4\n");
      printf("  cols: 4\n");
      printf("  data: [\n");
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
      printf("  ]\n");
    }
  }
}

/**
 * Add camera to camera calibration problem
 */
void calib_camera_add_camera(calib_camera_t *calib,
                             const int cam_idx,
                             const int cam_res[2],
                             const char *proj_model,
                             const char *dist_model,
                             const real_t *cam_params,
                             const real_t *cam_ext) {
  assert(calib != NULL);
  assert(cam_idx <= calib->num_cams);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(cam_params != NULL);
  assert(cam_ext != NULL);

  if (cam_idx > (calib->num_cams - 1)) {
    const int new_size = calib->num_cams + 1;
    calib->cam_params =
        realloc(calib->cam_params, sizeof(camera_params_t) * new_size);
    calib->cam_exts = realloc(calib->cam_exts, sizeof(extrinsic_t) * new_size);
  }

  camera_params_setup(&calib->cam_params[cam_idx],
                      cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_params);
  extrinsic_setup(&calib->cam_exts[cam_idx], cam_ext);
  if (cam_idx == 0) {
    calib->cam_exts[0].fix = 1;
  }

  calib->num_cams++;
  calib->cams_ok = 1;
}

// /**
//  * Add camera calibration view.
//  */
// void calib_camera_add_view(calib_camera_t *calib,
//                            const timestamp_t ts,
//                            const int view_idx,
//                            const int cam_idx,
//                            const int num_corners,
//                            const int *tag_ids,
//                            const int *corner_indices,
//                            const real_t *object_points,
//                            const real_t *keypoints) {
//   assert(calib != NULL);
//   assert(calib->cams_ok);
//   if (num_corners == 0) {
//     return;
//   }
//
//   // Pose T_C0F
//   pose_t *pose = hmgets(calib->poses, ts).value;
//   if (pose == NULL) {
//     // Estimate relative pose T_CiF
//     real_t T_CiF[4 * 4] = {0};
//     const int status = solvepnp_camera(&calib->cam_params[cam_idx],
//                                        keypoints,
//                                        object_points,
//                                        num_corners,
//                                        T_CiF);
//     if (status != 0) {
//       return;
//     }
//
//     // Form T_BF
//     POSE2TF(calib->cam_exts[cam_idx].data, T_BCi);
//     TF_CHAIN(T_BF, 2, T_BCi, T_CiF);
//     TF_VECTOR(T_BF, pose_vector);
//
//     // New pose
//     arrput(calib->timestamps, ts);
//     pose = malloc(sizeof(pose_t) * 1);
//     pose_setup(pose, ts, pose_vector);
//     hmput(calib->poses, ts, pose);
//   }
//
//   // Form new view
//   calib_frame_t **cam_views = hmgets(calib->view_sets, ts).value;
//   if (cam_views == NULL) {
//     cam_views = calloc(calib->num_cams, sizeof(calib_frame_t **));
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       cam_views[cam_idx] = NULL;
//     }
//     hmput(calib->view_sets, ts, cam_views);
//     calib->num_views++;
//   }
//
//   calib_frame_t *view =
//       calib_frame_malloc(ts,
//                                view_idx,
//                                cam_idx,
//                                num_corners,
//                                tag_ids,
//                                corner_indices,
//                                object_points,
//                                keypoints,
//                                pose,
//                                &calib->cam_exts[cam_idx],
//                                &calib->cam_params[cam_idx]);
//   cam_views[cam_idx] = view;
//   calib->num_factors += num_corners;
// }
//
// void calib_camera_marginalize(calib_camera_t *calib) {
//   // Setup marginalization factor
//   marg_factor_t *marg = marg_factor_malloc();
//
//   // Get first timestamp
//   const timestamp_t ts = calib->timestamps[0];
//
//   // Mark the pose at timestamp to be marginalized
//   pose_t *pose = hmgets(calib->poses, ts).value;
//   pose->marginalize = 1;
//
//   // Add calib camera factors to marginalization factor
//   calib_frame_t **cam_views = hmgets(calib->view_sets, ts).value;
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     calib_frame_t *view = cam_views[cam_idx];
//     if (view == NULL) {
//       continue;
//     }
//
//     for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//       marg_factor_add(marg, CALIB_CAMERA_FACTOR, &view->factors[factor_idx]);
//     }
//   }
//
//   // Add previous marginalization factor to new marginalization factor
//   if (calib->marg) {
//     marg_factor_add(marg, MARG_FACTOR, calib->marg);
//   }
//
//   // Marginalize
//   marg_factor_marginalize(marg);
//   if (calib->marg) {
//     marg_factor_free(calib->marg);
//   }
//   calib->marg = marg;
//
//   // Remove viewset
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     calib_frame_free(cam_views[cam_idx]);
//   }
//   free(cam_views);
//   (void) hmdel(calib->view_sets, ts);
//   // ^ (void) cast required for now: https://github.com/nothings/stb/issues/1574
//
//   // Remove timestamp
//   arrdel(calib->timestamps, 0);
//
//   // Update number of views
//   calib->num_views--;
// }

// /**
//  * Add camera calibration data.
//  */
// int calib_camera_add_data(calib_camera_t *calib,
//                           const int cam_idx,
//                           const char *data_path) {
//   // Get camera data
//   int num_files = 0;
//   char **files = list_files(data_path, &num_files);
//
//   // Exit if no calibration data
//   if (num_files == 0) {
//     for (int view_idx = 0; view_idx < num_files; view_idx++) {
//       free(files[view_idx]);
//     }
//     free(files);
//     return -1;
//   }
//
//   for (int view_idx = 0; view_idx < num_files; view_idx++) {
//     // Load aprilgrid
//     aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
//
//     // Get aprilgrid measurements
//     const timestamp_t ts = grid->timestamp;
//     const int num_corners = grid->corners_detected;
//     int *tag_ids = malloc(sizeof(int) * num_corners);
//     int *corner_indices = malloc(sizeof(int) * num_corners);
//     real_t *kps = malloc(sizeof(real_t) * num_corners * 2);
//     real_t *pts = malloc(sizeof(real_t) * num_corners * 3);
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
//     // Clean up
//     free(tag_ids);
//     free(corner_indices);
//     free(kps);
//     free(pts);
//     free(files[view_idx]);
//     aprilgrid_free(grid);
//   }
//   free(files);
//
//   return 0;
// }
//
// int calib_camera_shannon_entropy(calib_camera_t *calib, real_t *entropy) {
//   // Determine parameter order
//   int sv_size = 0;
//   int r_size = 0;
//   param_order_t *hash = calib_camera_param_order(calib, &sv_size, &r_size);
//
//   // Form Hessian H
//   real_t *H = calloc(sv_size * sv_size, sizeof(real_t));
//   real_t *g = calloc(sv_size, sizeof(real_t));
//   real_t *r = calloc(r_size, sizeof(real_t));
//   calib_camera_linearize_compact(calib, sv_size, hash, H, g, r);
//
//   // Estimate covariance
//   real_t *covar = calloc(sv_size * sv_size, sizeof(real_t));
//   pinv(H, sv_size, sv_size, covar);
//
//   // Grab the rows and columns corresponding to calib parameters
//   // In the following we assume the state vector x is ordered:
//   //
//   //   x = [ poses [1..k], N camera extrinsics, N camera parameters]
//   //
//   // We are only interested in the Shannon-Entropy, or the uncertainty of the
//   // calibration parameters. In this case the N camera extrinsics and
//   // parameters, so once we have formed the full Hessian H matrix, inverted it
//   // to form the covariance matrix, we can extract the lower right block matrix
//   // that corresponds to the uncertainty of the calibration parameters, then
//   // use it to calculate the shannon entropy.
//   const timestamp_t last_ts = calib->timestamps[calib->num_views - 1];
//   void *data = hmgets(calib->poses, last_ts).value->data;
//   const int idx_s = hmgets(hash, data).idx + 6;
//   const int idx_e = sv_size - 1;
//   const int m = idx_e - idx_s + 1;
//   real_t *covar_params = calloc(m * m, sizeof(real_t));
//   mat_block_get(covar, sv_size, idx_s, idx_e, idx_s, idx_e, covar_params);
//
//   // Calculate shannon-entropy
//   int status = 0;
//   if (shannon_entropy(covar_params, m, entropy) != 0) {
//     status = -1;
//   }
//
//   // Clean up
//   hmfree(hash);
//   free(covar_params);
//   free(covar);
//   free(H);
//   free(g);
//   free(r);
//
//   return status;
// }
//
// /**
//  * Camera calibration parameter order.
//  */
// param_order_t *calib_camera_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size) {
//   // Setup parameter order
//   calib_camera_t *calib = (calib_camera_t *) data;
//   param_order_t *hash = NULL;
//   int col_idx = 0;
//
//   // -- Add body poses
//   for (int i = 0; i < hmlen(calib->poses); i++) {
//     param_order_add_pose(&hash, calib->poses[i].value, &col_idx);
//   }
//
//   // -- Add camera extrinsic
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_extrinsic(&hash, &calib->cam_exts[cam_idx], &col_idx);
//   }
//
//   // -- Add camera parameters
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_camera(&hash, &calib->cam_params[cam_idx], &col_idx);
//   }
//
//   // Set state-vector and residual size
//   *sv_size = col_idx;
//   *r_size = (calib->num_factors * 2);
//   if (calib->marg) {
//     *r_size += calib->marg->r_size;
//   }
//
//   return hash;
// }
//
// /**
//  * Calculate camera calibration problem cost.
//  */
// void calib_camera_cost(const void *data, real_t *r) {
//   // Evaluate factors
//   calib_camera_t *calib = (calib_camera_t *) data;
//
//   // -- Evaluate calib camera factors
//   int r_idx = 0;
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_frame_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         struct calib_camera_factor_t *factor = &view->factors[factor_idx];
//         calib_camera_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // -- Evaluate marginalization factor
//   if (calib->marg) {
//     marg_factor_eval(calib->marg);
//     vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//   }
// }
//
// /**
//  * Linearize camera calibration problem.
//  */
// void calib_camera_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r) {
//   // Evaluate factors
//   calib_camera_t *calib = (calib_camera_t *) data;
//   int r_idx = 0;
//
//   // -- Evaluate calib camera factors
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_frame_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         struct calib_camera_factor_t *factor = &view->factors[factor_idx];
//         calib_camera_factor_eval(factor);
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
//   // -- Evaluate marginalization factor
//   if (calib->marg) {
//     marg_factor_eval(calib->marg);
//     vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//
//     solver_fill_hessian(hash,
//                         calib->marg->num_params,
//                         calib->marg->params,
//                         calib->marg->jacs,
//                         calib->marg->r,
//                         calib->marg->r_size,
//                         sv_size,
//                         H,
//                         g);
//   }
// }

/**
 * Reduce camera calibration problem via Schur-Complement.
 *
 * The Gauss newton system we are trying to solve has the form:
 *
 *   H dx = b (1)
 *
 * Where the H is the Hessian, dx is the update vector and b is a vector. In the
 * camera calibration problem the Hessian has a arrow head pattern (see (25) in
 * [Triggs2000]). This means to avoid inverting the full H matrix we can
 * decompose (1) as,
 *
 *   [A B * [dx0    [b0
 *    C D]   dx1] =  b1]  (2)
 *
 * and take the Shur-complement of A, we get a reduced system of:
 *
 *   D_bar = D − C * A^-1 * B
 *   b1_bar = b1 − C * A^-1 * b0  (3)
 *
 * Since A is a block diagonal, inverting it is much cheaper than inverting the
 * full H or A matrix. With (3) we can solve for dx1.
 *
 *   D_bar * dx1 = b1_bar
 *
 * And finally back-substitute the newly estimated dx1 to find dx0,
 *
 *   A * dx0 = b0 - B * dx1
 *   dx0 = A^-1 * b0 - B * dx1
 *
 * where in the previous steps we have already computed A^-1.
 *
 * [Triggs2000]:
 *
 *   Triggs, Bill, et al. "Bundle adjustment—a modern synthesis." Vision
 *   Algorithms: Theory and Practice: International Workshop on Vision
 *   Algorithms Corfu, Greece, September 21–22, 1999 Proceedings. Springer
 *   Berlin Heidelberg, 2000.
 *
 */
// void calib_camera_linsolve(const void *data,
//                            const int sv_size,
//                            param_order_t *hash,
//                            real_t *H,
//                            real_t *g,
//                            real_t *dx) {
//   calib_camera_t *calib = (calib_camera_t *) data;
//   const int m = calib->num_views * 6;
//   const int r = sv_size - m;
//   const int H_size = sv_size;
//   const int bs = 6; // Diagonal block size
//
//   // Extract sub-blocks of matrix H
//   // H = [A, B,
//   //      C, D]
//   real_t *B = malloc(sizeof(real_t) * m * r);
//   real_t *C = malloc(sizeof(real_t) * r * m);
//   real_t *D = malloc(sizeof(real_t) * r * r);
//   real_t *A_inv = malloc(sizeof(real_t) * m * m);
//   mat_block_get(H, H_size, 0, m - 1, m, H_size - 1, B);
//   mat_block_get(H, H_size, m, H_size - 1, 0, m - 1, C);
//   mat_block_get(H, H_size, m, H_size - 1, m, H_size - 1, D);
//
//   // Extract sub-blocks of vector b
//   // b = [b0, b1]
//   real_t *b0 = malloc(sizeof(real_t) * m);
//   real_t *b1 = malloc(sizeof(real_t) * r);
//   vec_copy(g, m, b0);
//   vec_copy(g + m, r, b1);
//
//   // Invert A
//   bdiag_inv_sub(H, sv_size, m, bs, A_inv);
//
//   // Reduce H * dx = b with Shur-Complement
//   // D_bar = D - C * A_inv * B
//   // b1_bar = b1 - C * A_inv * b0
//   real_t *D_bar = malloc(sizeof(real_t) * r * r);
//   real_t *b1_bar = malloc(sizeof(real_t) * r * 1);
//   dot3(C, r, m, A_inv, m, m, B, m, r, D_bar);
//   dot3(C, r, m, A_inv, m, m, b0, m, 1, b1_bar);
//   for (int i = 0; i < (r * r); i++) {
//     D_bar[i] = D[i] - D_bar[i];
//   }
//   for (int i = 0; i < r; i++) {
//     b1_bar[i] = b1[i] - b1_bar[i];
//   }
//
//   // Solve reduced system: D_bar * dx_r = b1_bar
//   real_t *dx_r = malloc(sizeof(real_t) * r * 1);
//   // Hack: precondition D_bar so linear-solver doesn't complain
//   for (int i = 0; i < r; i++) {
//     D_bar[i * r + i] += 1e-4;
//   }
//   chol_solve(D_bar, b1_bar, dx_r, r);
//
//   // Back-subsitute
//   real_t *B_dx_r = calloc(m * 1, sizeof(real_t));
//   real_t *dx_m = calloc(m * 1, sizeof(real_t));
//   dot(B, m, r, dx_r, r, 1, B_dx_r);
//   for (int i = 0; i < m; i++) {
//     b0[i] = b0[i] - B_dx_r[i];
//   }
//   bdiag_dot(A_inv, m, m, bs, b0, dx_m);
//
//   // Form full dx vector
//   for (int i = 0; i < m; i++) {
//     dx[i] = dx_m[i];
//   }
//   for (int i = 0; i < r; i++) {
//     dx[i + m] = dx_r[i];
//   }
//
//   // Clean-up
//   free(B);
//   free(C);
//   free(D);
//   free(A_inv);
//
//   free(b0);
//   free(b1);
//
//   free(D_bar);
//   free(b1_bar);
//
//   free(B_dx_r);
//   free(dx_m);
//   free(dx_r);
// }

/**
 * Solve camera calibration problem.
 */
// void calib_camera_solve(calib_camera_t *calib) {
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
//   solver.cost_func = &calib_camera_cost;
//   solver.param_order_func = &calib_camera_param_order;
//   solver.linearize_func = &calib_camera_linearize_compact;
//   // solver.linsolve_func = &calib_camera_linsolve;
//   solver_solve(&solver, calib);
//
//   if (calib->verbose) {
//     calib_camera_print(calib);
//   }
// }
