#include "munit.h"
#include "xyz.h"
#include "xyz_se.h"
#include "xyz_calib.h"
#include "xyz_aprilgrid.h"
#include "xyz_sim.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"
#define TEST_SIM_GIMBAL TEST_DATA_PATH "sim_gimbal"
#define TEST_CAM_APRIL TEST_DATA_PATH "cam_april"
#define TEST_IMU_APRIL TEST_DATA_PATH "imu_april"

int test_schur_complement(void) {
  // clang-format off
  real_t H[10 * 10] = {
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
  };
  real_t b[10] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
  // clang-format on

  // H = [Hmm, Hmr,
  //      Hrm, Hrr]
  real_t Hmm[4 * 4] = {0};
  real_t Hmr[4 * 6] = {0};
  real_t Hrm[6 * 4] = {0};
  real_t Hrr[6 * 6] = {0};
  int H_size = 10;
  int m = 4;
  int r = 6;
  mat_block_get(H, H_size, 0, m - 1, 0, m - 1, Hmm);
  mat_block_get(H, H_size, 0, m - 1, m, H_size - 1, Hmr);
  mat_block_get(H, H_size, m, H_size - 1, 0, m - 1, Hrm);
  mat_block_get(H, H_size, m, H_size - 1, m, H_size - 1, Hrr);

  // print_matrix("H", H, 10, 10);
  // print_matrix("Hmm", Hmm, m, m);
  // print_matrix("Hmr", Hmr, m, r);
  // print_matrix("Hrm", Hrm, r, m);
  // print_matrix("Hrr", Hrr, r, r);

  real_t bmm[4] = {0};
  real_t brr[6] = {0};
  vec_copy(b, m, bmm);
  vec_copy(b + m, r, brr);

  // print_vector("b", b, 10);
  // print_vector("bmm", bmm, m);
  // print_vector("brr", brr, r);

  // real_t *Hmm = MALLOC(real_t, m * m);
  // real_t *Hmr = MALLOC(real_t, m * r);
  // real_t *Hrm = MALLOC(real_t, m * r);
  // real_t *Hrr = MALLOC(real_t, r * r);
  // real_t *Hmm_inv = MALLOC(real_t, m * m);

  return 0;
}

int test_timeline(void) {
  const char *data_dir = TEST_IMU_APRIL;
  const int num_cams = 2;
  const int num_imus = 1;
  timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
  // printf("timeline->num_cams: %d\n", timeline->num_cams);
  // printf("timeline->num_imus: %d\n", timeline->num_imus);
  // printf("timeline->num_event_types: %d\n", timeline->num_event_types);

  FILE *imu_file = fopen("/tmp/imu.csv", "w");

  for (int k = 0; k < timeline->timeline_length; k++) {
    // Extract timeline events. Add either imu or fiducial event
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      // const timestamp_t ts = event->ts;

      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        // printf("imu_ts: %ld ", data->ts);
        // printf("acc: (%f, %f, %f) ", data->acc[0], data->acc[1], data->acc[2]);
        // printf("gyr: (%f, %f, %f) ", data->gyr[0], data->gyr[1], data->gyr[2]);
        // printf("\n");

        fprintf(imu_file, "%ld,", data->ts);
        fprintf(imu_file,
                "%lf,%lf,%lf,",
                data->gyr[0],
                data->gyr[1],
                data->gyr[2]);
        fprintf(imu_file,
                "%lf,%lf,%lf",
                data->acc[0],
                data->acc[1],
                data->acc[2]);
        fprintf(imu_file, "\n");

      } else if (event->type == FIDUCIAL_EVENT) {
        // const fiducial_event_t *data = &event->data.fiducial;
        // const int cam_idx = data->cam_idx;
        // printf("cam_ts: %ld \n", data->ts);
        // printf("  cam_idx: %d\n", data->cam_idx);
        // printf("  num_corners: %d\n", data->num_corners);
        // for (int i = 0; i < data->num_corners; i++) {
        //   const real_t *p = data->object_points + i * 3;
        //   const real_t *z = data->keypoints + i * 2;

        //   printf("  ");
        //   printf("%d, ", data->tag_ids[i]);
        //   printf("%d, ", data->corner_indices[i]);
        //   printf("%f, %f, %f, ", p[0], p[1], p[2]);
        //   printf("%f, %f", z[0], z[1]);
        //   printf("\n");
        // }
      }
    }
  }

  // Clean up
  timeline_free(timeline);
  fclose(imu_file);

  return 0;
}

int test_pose(void) {
  timestamp_t ts = 1;
  pose_t pose;

  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 1.1, 2.2, 3.3};
  pose_setup(&pose, ts, data);

  MU_ASSERT(pose.ts == 1);

  MU_ASSERT(fltcmp(pose.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[2], 0.3) == 0.0);
  MU_ASSERT(fltcmp(pose.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(pose.data[4], 1.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[5], 2.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[6], 3.3) == 0.0);

  return 0;
}

int test_extrinsics(void) {
  extrinsic_t extrinsic;

  real_t data[7] = {1.0, 2.0, 3.0, 1.0, 0.1, 0.2, 0.3};
  extrinsic_setup(&extrinsic, data);

  MU_ASSERT(fltcmp(extrinsic.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[2], 3.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[4], 0.1) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[5], 0.2) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[6], 0.3) == 0.0);

  return 0;
}

int test_fiducial(void) {
  fiducial_t fiducial;

  real_t data[7] = {1.0, 2.0, 3.0, 1.0, 0.1, 0.2, 0.3};
  fiducial_setup(&fiducial, data);

  MU_ASSERT(fltcmp(fiducial.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[2], 3.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[4], 0.1) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[5], 0.2) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[6], 0.3) == 0.0);

  return 0;
}

// int test_fiducial_buffer(void) {
//   const timestamp_t ts = 0;
//   const int cam_idx = 1;
//   const int n = 2;
//   const int tag_ids[2] = {1, 2};
//   const int corner_idxs[2] = {1, 2};
//   const real_t pts[2 * 3] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
//   const real_t kps[2 * 2] = {0.0, 0.0, 1.0, 1.0};
//
//   fiducial_buffer_t *buf = fiducial_buffer_malloc();
//   fiducial_buffer_add(buf, ts, cam_idx, n, tag_ids, corner_idxs, pts, kps);
//
//   MU_ASSERT(buf->data[0]->ts == ts);
//   MU_ASSERT(buf->data[0]->cam_idx == cam_idx);
//   MU_ASSERT(buf->data[0]->num_corners == n);
//   MU_ASSERT(vec_equals(buf->data[0]->object_points, pts, n * 3) == 1);
//   MU_ASSERT(vec_equals(buf->data[0]->keypoints, kps, n * 2) == 1);
//   MU_ASSERT(buf->size == 1);
//   MU_ASSERT(buf->capacity > 1);
//
//   fiducial_buffer_free(buf);
//
//   return 0;
// }

int test_imu_biases(void) {
  timestamp_t ts = 1;
  imu_biases_t biases;

  real_t ba[3] = {1.0, 2.0, 3.0};
  real_t bg[3] = {4.0, 5.0, 6.0};
  imu_biases_setup(&biases, ts, ba, bg);

  MU_ASSERT(biases.ts == 1);

  MU_ASSERT(fltcmp(biases.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[2], 3.0) == 0.0);

  MU_ASSERT(fltcmp(biases.data[3], 4.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[4], 5.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[5], 6.0) == 0.0);

  return 0;
}

int test_feature(void) {
  feature_t feature;

  size_t feature_id = 99;
  real_t data[3] = {0.1, 0.2, 0.3};
  feature_init(&feature, feature_id, data);

  printf("sizeof(feature_t): %ld bytes\n", sizeof(feature_t));

  MU_ASSERT(feature.feature_id == feature_id);
  MU_ASSERT(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

// int test_idf(void) {
//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, 0, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);
//   TF_ROT(T_WCi, C_WCi);
//   TF_TRANS(T_WCi, r_WCi);

//   const size_t feature_id = 0;
//   const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//   real_t z[2] = {0};
//   TF_POINT(T_CiW, p_W, p_Ci);
//   pinhole_radtan4_project(cam_data, p_Ci, z);

//   // Setup IDF
//   pos_t idf_pos;
//   pos_setup(&idf_pos, r_WCi);

//   feature_t idf_param;
//   idf_setup(&idf_param, feature_id, 0, &cam, C_WCi, z);

//   // Reproject IDF to feature in world frame
//   real_t p_W_est[3] = {0};
//   idf_point(&idf_param, idf_pos.data, p_W_est);

//   const real_t dx = p_W_est[0] - p_W[0];
//   const real_t dy = p_W_est[1] - p_W[1];
//   const real_t dz = p_W_est[2] - p_W[2];
//   const real_t dist = sqrt(dx * dx + dy * dy + dz * dz);
//   MU_ASSERT(dist < 1e-1);

//   return 0;
// }

// int test_features(void) {
//   // XYZ Features
//   // clang-format off
//   size_t feature_ids[3] = {1, 2, 3};
//   real_t params[3 * 3] = {0.1, 0.2, 0.3,
//                           0.4, 0.5, 0.6,
//                           0.7, 0.8, 0.9};
//   // clang-format on

//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, 0, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);

//   // Setup inverse-depth features and keypoints
//   size_t num_idfs = 10;
//   size_t *idf_ids = MALLOC(size_t, num_idfs);
//   real_t *idf_features = MALLOC(real_t, num_idfs * 3);
//   real_t *idf_keypoints = MALLOC(real_t, num_idfs * 2);
//   for (size_t i = 0; i < num_idfs; i++) {
//     const size_t feature_id = i + (4);
//     const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//     real_t z[2] = {0};
//     TF_POINT(T_CiW, p_W, p_Ci);
//     pinhole_radtan4_project(cam_data, p_Ci, z);

//     idf_ids[i] = feature_id;
//     idf_features[i * 3 + 0] = p_W[0];
//     idf_features[i * 3 + 1] = p_W[1];
//     idf_features[i * 3 + 2] = p_W[2];
//     idf_keypoints[i * 2 + 0] = z[0];
//     idf_keypoints[i * 2 + 1] = z[1];
//   }

//   // Setup
//   features_t *features = features_malloc();

//   // -- Add XYZ features
//   features_add_xyzs(features, feature_ids, params, 3);
//   MU_ASSERT(features->num_features == 3);

//   // // -- Add IDF features
//   // features_add_idfs(features, idf_ids, &cam, T_WCi, idf_keypoints, num_idfs);
//   // MU_ASSERT(features->num_features == 3 + num_idfs);

//   // -- Check features exists
//   MU_ASSERT(features_exists(features, 1) == 1);
//   MU_ASSERT(features_exists(features, 2) == 1);
//   MU_ASSERT(features_exists(features, 3) == 1);
//   MU_ASSERT(features_exists(features, 99) == 0);

//   // -- Get features
//   feature_t *f0 = NULL;
//   feature_t *f1 = NULL;
//   feature_t *f2 = NULL;
//   feature_t *f3 = NULL;

//   features_get_xyz(features, 1, &f0);
//   features_get_xyz(features, 2, &f1);
//   features_get_xyz(features, 3, &f2);
//   features_get_xyz(features, 99, &f3);

//   MU_ASSERT(f0->feature_id == 1);
//   MU_ASSERT(f0->status == 1);
//   MU_ASSERT(vec_equals(f0->data, params + 0, 3) == 1);

//   MU_ASSERT(f1->feature_id == 2);
//   MU_ASSERT(f1->status == 1);
//   MU_ASSERT(vec_equals(f1->data, params + 3, 3) == 1);

//   MU_ASSERT(f2->feature_id == 3);
//   MU_ASSERT(f2->status == 1);
//   MU_ASSERT(vec_equals(f2->data, params + 6, 3) == 1);

//   MU_ASSERT(f3 == NULL);

//   // Clean up
//   features_free(features);
//   free(idf_ids);
//   free(idf_features);
//   free(idf_keypoints);

//   return 0;
// }

int test_time_delay(void) {
  time_delay_t td;
  time_delay_setup(&td, 1.0);
  MU_ASSERT(fltcmp(td.data[0], 1.0) == 0);
  return 0;
}

int test_joint(void) {
  joint_t joint;
  joint_setup(&joint, 101, 2, 1.0);
  MU_ASSERT(joint.ts == 101);
  MU_ASSERT(joint.joint_idx == 2);
  MU_ASSERT(fltcmp(joint.data[0], 1.0) == 0);
  return 0;
}

int test_camera_params(void) {
  camera_params_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&camera, cam_idx, cam_res, proj_model, dist_model, data);
  // camera_params_print(&camera);

  return 0;
}

int test_triangulation_batch(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const int cam_res[2] = {image_width, image_height};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  const real_t data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_i;
  camera_params_t cam_j;
  camera_params_setup(&cam_i, 0, cam_res, pmodel, dmodel, data);
  camera_params_setup(&cam_j, 1, cam_res, pmodel, dmodel, data);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup camera extrinsics T_CiCj
  TF_INV(T_WC0, T_C0W);
  TF_CHAIN(T_CiCj, 2, T_C0W, T_WC1);

  // Setup 3D and 2D correspondance points
  int N = 10;
  real_t *kps_i = MALLOC(real_t, N * 2);
  real_t *kps_j = MALLOC(real_t, N * 2);
  real_t *points_gnd = MALLOC(real_t, N * 3);
  real_t *points_est = MALLOC(real_t, N * 3);
  int *status = MALLOC(int, N);

  for (int i = 0; i < N; i++) {
    const real_t p_W[3] = {5.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);

    kps_i[i * 2 + 0] = z0[0];
    kps_i[i * 2 + 1] = z0[1];

    kps_j[i * 2 + 0] = z1[0];
    kps_j[i * 2 + 1] = z1[1];

    points_gnd[i * 3 + 0] = p_C0[0];
    points_gnd[i * 3 + 1] = p_C0[1];
    points_gnd[i * 3 + 2] = p_C0[2];
  }

  // Test triangulate batch
  triangulate_batch(&cam_i,
                    &cam_j,
                    T_CiCj,
                    kps_i,
                    kps_j,
                    N,
                    points_est,
                    status);
  for (int i = 0; i < N; i++) {
    const real_t *p_gnd = points_gnd + i * 3;
    const real_t *p_est = points_est + i * 3;
    const real_t dx = p_gnd[0] - p_est[0];
    const real_t dy = p_gnd[1] - p_est[1];
    const real_t dz = p_gnd[2] - p_est[2];
    const real_t diff = sqrt(dx * dx + dy * dy + dz * dz);

    MU_ASSERT(diff < 0.01);
    // printf("gnd: (%.2f, %.2f, %.2f), ", p_gnd[0], p_gnd[1], p_gnd[2]);
    // printf("est: (%.2f, %.2f, %.2f), ", p_est[0], p_est[1], p_est[2]);
    // printf("diff: %.2e\n", diff);
  }

  // Clean up
  free(kps_i);
  free(kps_j);
  free(points_gnd);
  free(points_est);
  free(status);

  return 0;
}

int test_pose_factor(void) {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&factor, &pose, var);

  /* Check Jacobians */
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, pose_factor_eval, step_size, tol, 0);

  return 0;
}

int test_ba_factor(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Camera pose
  const real_t pose_data[7] = {0.01, 0.01, 0.0, 0.5, -0.5, 0.5, -0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  // Feature
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  feature_t feature;
  feature_init(&feature, 0, p_W);

  // Camera parameters
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.03, 0.01, 0.001, 0.001};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0.0};
  real_t z[2] = {0.0};
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  // Bundle adjustment factor
  ba_factor_t factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&factor, &pose, &feature, &cam, z, var);

  // Check Jacobians
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, ba_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, ba_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, ba_factor_eval, step_size, tol, 0);

  return 0;
}

int test_camera_factor(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Body pose T_WB
  pose_t pose;
  const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, pose_data);

  // Extrinsic T_BC
  extrinsic_t cam_ext;
  const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsic_setup(&cam_ext, ext_data);

  // Feature p_W
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_init(&feature, 0, p_W);

  // Camera parameters
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t z[2];
  TF(pose_data, T_WB);
  TF(ext_data, T_BCi);
  TF_INV(T_WB, T_BW);
  TF_INV(T_BCi, T_CiB);
  DOT(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  TF_POINT(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  // Setup camera factor
  camera_factor_t factor;
  real_t var[2] = {1.0, 1.0};
  camera_factor_setup(&factor, &pose, &cam_ext, &feature, &cam, z, var);

  // Check Jacobians
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(3, factor, camera_factor_eval, step_size, tol, 0);

  return 0;
}

// int test_idf_factor(void) {
//   // Timestamp
//   timestamp_t ts = 0;

//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, ts, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);
//   TF_TRANS(T_WCi, r_WCi);

//   const size_t feature_id = 0;
//   const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//   real_t z[2] = {0};
//   TF_POINT(T_CiW, p_W, p_Ci);
//   pinhole_radtan4_project(cam_data, p_Ci, z);

//   // Setup IDF
//   pos_t idf_pos;
//   pos_setup(&idf_pos, r_WCi);

//   feature_t idf_param;
//   TF_ROT(T_WCi, C_WCi);
//   idf_setup(&idf_param, feature_id, 0, &cam, C_WCi, z);

//   // Setup IDF Factor
//   const real_t var[2] = {1.0, 1.0};
//   idf_factor_t factor;
//   idf_factor_setup(&factor,
//                    &pose,
//                    &cam_ext,
//                    &cam,
//                    &idf_pos,
//                    &idf_param,
//                    ts,
//                    cam_idx,
//                    feature_id,
//                    z,
//                    var);

//   // Check Jacobians
//   const real_t step_size = 1e-8;
//   const real_t tol = 1e-4;
//   const int debug = 0;
//   CHECK_FACTOR_J(0, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(1, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(2, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(3, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(4, factor, idf_factor_eval, step_size, tol, debug);

//   return 0;
// }

int test_imu_buffer_setup(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  return 0;
}

int test_imu_buffer_add(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);

  MU_ASSERT(imu_buf.size == 1);
  MU_ASSERT(imu_buf.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 3.0) == 0);

  return 0;
}

int test_imu_buffer_clear(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);
  imu_buffer_clear(&imu_buf);

  MU_ASSERT(imu_buf.size == 0);
  MU_ASSERT(imu_buf.ts[0] == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 0.0) == 0);

  return 0;
}

int test_imu_buffer_copy(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);

  imu_buffer_t imu_buf2;
  imu_buffer_setup(&imu_buf2);
  imu_buffer_copy(&imu_buf, &imu_buf2);

  MU_ASSERT(imu_buf2.size == 1);
  MU_ASSERT(imu_buf2.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][2], 3.0) == 0);

  return 0;
}

typedef struct imu_test_data_t {
  size_t num_measurements;
  real_t *timestamps;
  real_t **poses;
  real_t **velocities;
  real_t **imu_acc;
  real_t **imu_gyr;
} imu_test_data_t;

static int setup_imu_test_data(imu_test_data_t *test_data,
                               const real_t circle_r,
                               const real_t circle_v) {
  // Circle trajectory configurations
  const real_t imu_rate = 200.0;
  const real_t circle_dist = 2.0 * M_PI * circle_r;
  const real_t time_taken = circle_dist / circle_v;
  const real_t w = -2.0 * M_PI * (1.0 / time_taken);
  const real_t theta_init = M_PI;
  const real_t yaw_init = M_PI / 2.0;

  // Allocate memory for test data
  test_data->num_measurements = time_taken * imu_rate;
  test_data->timestamps = CALLOC(real_t, test_data->num_measurements);
  test_data->poses = CALLOC(real_t *, test_data->num_measurements);
  test_data->velocities = CALLOC(real_t *, test_data->num_measurements);
  test_data->imu_acc = CALLOC(real_t *, test_data->num_measurements);
  test_data->imu_gyr = CALLOC(real_t *, test_data->num_measurements);

  // Simulate IMU poses
  const real_t dt = 1.0 / imu_rate;
  timestamp_t ts = 0.0;
  real_t theta = theta_init;
  real_t yaw = yaw_init;

  for (size_t k = 0; k < test_data->num_measurements; k++) {
    // IMU pose
    // -- Position
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    // -- Orientation
    const real_t ypr[3] = {yaw, 0.0, 0.0};
    real_t q[4] = {0};
    euler2quat(ypr, q);
    // -- Pose vector
    const real_t pose[7] = {rx, ry, rz, q[0], q[1], q[2], q[3]};
    // print_vector("pose", pose, 7);

    // Velocity
    const real_t vx = -circle_r * w * sin(theta);
    const real_t vy = circle_r * w * cos(theta);
    const real_t vz = 0.0;
    const real_t v_WS[3] = {vx, vy, vz};

    // Acceleration
    const real_t ax = -circle_r * w * w * cos(theta);
    const real_t ay = -circle_r * w * w * sin(theta);
    const real_t az = 0.0;
    const real_t a_WS[3] = {ax, ay, az};

    // Angular velocity
    const real_t wx = 0.0;
    const real_t wy = 0.0;
    const real_t wz = w;
    const real_t w_WS[3] = {wx, wy, wz};

    // IMU measurements
    real_t C_WS[3 * 3] = {0};
    real_t C_SW[3 * 3] = {0};
    quat2rot(q, C_WS);
    mat_transpose(C_WS, 3, 3, C_SW);
    // -- Accelerometer measurement
    real_t acc[3] = {0};
    dot(C_SW, 3, 3, a_WS, 3, 1, acc);
    acc[2] += 9.81;
    // -- Gyroscope measurement
    real_t gyr[3] = {0};
    dot(C_SW, 3, 3, w_WS, 3, 1, gyr);

    // Update
    test_data->timestamps[k] = ts;
    test_data->poses[k] = vector_malloc(pose, 7);
    test_data->velocities[k] = vector_malloc(v_WS, 3);
    test_data->imu_acc[k] = vector_malloc(acc, 3);
    test_data->imu_gyr[k] = vector_malloc(gyr, 3);

    theta += w * dt;
    yaw += w * dt;
    ts += sec2ts(dt);
  }

  return 0;
}

static void free_imu_test_data(imu_test_data_t *test_data) {
  for (size_t k = 0; k < test_data->num_measurements; k++) {
    free(test_data->poses[k]);
    free(test_data->velocities[k]);
    free(test_data->imu_acc[k]);
    free(test_data->imu_gyr[k]);
  }

  free(test_data->timestamps);
  free(test_data->poses);
  free(test_data->velocities);
  free(test_data->imu_acc);
  free(test_data->imu_gyr);
}

int test_imu_propagate(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Setup IMU buffer
  const int n = 100;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < n; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Test imu propagate
  real_t pose_k[7] = {0};
  real_t vel_k[3] = {0};
  real_t pose_kp1[7] = {0};
  real_t vel_kp1[3] = {0};

  vec_copy(test_data.poses[0], 7, pose_k);
  vec_copy(test_data.velocities[0], 3, vel_k);
  imu_propagate(pose_k, vel_k, &imu_buf, pose_kp1, vel_kp1);

  MU_PRINT_VECTOR("[gnd] pose_kp1", test_data.poses[n], 7);
  MU_PRINT_VECTOR("[est] pose_kp1", pose_kp1, 7);
  MU_PRINT_VECTOR("[gnd] vel_kp1", test_data.velocities[n], 3);
  MU_PRINT_VECTOR("[est] vel_kp1", vel_kp1, 3);

  real_t dr[3] = {0};
  real_t dtheta = 0;
  pose_diff2(test_data.poses[n], pose_kp1, dr, &dtheta);

  const real_t tol = 1e-3;
  MU_ASSERT(fabs(dr[0]) < tol);
  MU_ASSERT(fabs(dr[1]) < tol);
  MU_ASSERT(fabs(dr[2]) < tol);
  MU_ASSERT(fabs(dtheta) < tol);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_initial_attitude(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 5.0, 1.0);

  // Setup IMU buffer
  const int n = 1;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < n; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Test imu initial attitude
  real_t q_WS[4] = {0};
  imu_initial_attitude(&imu_buf, q_WS);
  MU_PRINT_VECTOR("[gnd] q_WS", test_data.poses[0], 7);
  MU_PRINT_VECTOR("[est] q_WS", q_WS, 4);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

// static void imu_propagate_step(const real_t x_km1[16], real_t x_k[16]) {
//   // Setup
//   const real_t a_i[3] = {0.1, 0.1, 0.1};
//   const real_t a_j[3] = {0.2, 0.2, 0.2};
//   const real_t w_i[3] = {0.1, 0.1, 0.1};
//   const real_t w_j[3] = {0.2, 0.2, 0.2};
//   const real_t dt = 0.01;
//   const real_t dt_sq = dt * dt;

//   const real_t *r_i = x_km1 + 0;
//   const real_t *q_i = x_km1 + 3;
//   const real_t *v_i = x_km1 + 7;
//   const real_t *ba_i = x_km1 + 10;
//   const real_t *bg_i = x_km1 + 13;

//   // Gyroscope measurement
//   const real_t wx = 0.5 * (w_i[0] + w_j[0]) - bg_i[0];
//   const real_t wy = 0.5 * (w_i[1] + w_j[1]) - bg_i[1];
//   const real_t wz = 0.5 * (w_i[2] + w_j[2]) - bg_i[2];
//   const real_t dq[4] = {1.0, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt};

//   // Update orientation
//   real_t q_j[4] = {0};
//   quat_mul(q_i, dq, q_j);
//   quat_normalize(q_j);

//   // Accelerometer measurement
//   const real_t a_ii[3] = {a_i[0] - ba_i[0], a_i[1] - ba_i[1], a_i[2] - ba_i[2]};
//   const real_t a_jj[3] = {a_j[0] - ba_i[0], a_j[1] - ba_i[1], a_j[2] - ba_i[2]};
//   real_t acc_i[3] = {0};
//   real_t acc_j[3] = {0};
//   quat_transform(q_i, a_ii, acc_i);
//   quat_transform(q_j, a_jj, acc_j);
//   real_t a[3] = {0};
//   a[0] = 0.5 * (acc_i[0] + acc_j[0]);
//   a[1] = 0.5 * (acc_i[1] + acc_j[1]);
//   a[2] = 0.5 * (acc_i[2] + acc_j[2]);

//   // Update position:
//   // r_j = r_i + (v_i * dt) + (0.5 * a * dt_sq)
//   real_t r_j[3] = {0};
//   r_j[0] = r_i[0] + (v_i[0] * dt) + (0.5 * a[0] * dt_sq);
//   r_j[1] = r_i[1] + (v_i[1] * dt) + (0.5 * a[1] * dt_sq);
//   r_j[2] = r_i[2] + (v_i[2] * dt) + (0.5 * a[2] * dt_sq);

//   // Update velocity:
//   // v_j = v_i + a * dt
//   real_t v_j[3] = {0};
//   v_j[0] = v_i[0] + a[0] * dt;
//   v_j[1] = v_i[1] + a[1] * dt;
//   v_j[2] = v_i[2] + a[2] * dt;

//   // Update biases
//   // ba_j = ba_i;
//   // bg_j = bg_i;
//   real_t ba_j[3] = {0};
//   real_t bg_j[3] = {0};
//   vec_copy(ba_i, 3, ba_j);
//   vec_copy(bg_i, 3, bg_j);

//   // Write outputs
//   imu_state_vector(r_j, q_j, v_j, ba_j, bg_j, x_k);
// }

int test_imu_factor_form_F_matrix(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Setup IMU buffer
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < 10; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0.0, 0.0, 0.0};
  const real_t bg_i[3] = {0.0, 0.0, 0.0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0.0, 0.0, 0.0};
  const real_t bg_j[3] = {0.0, 0.0, 0.0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  // imu_params_t imu_params;
  // imu_params.imu_idx = 0;
  // imu_params.rate = 200.0;
  // imu_params.sigma_a = 0.08;
  // imu_params.sigma_g = 0.004;
  // imu_params.sigma_aw = 0.00004;
  // imu_params.sigma_gw = 2.0e-6;
  // imu_params.g = 9.81;

  // Test form F Matrix
  const int k = idx_j;
  const real_t *q_i = pose_i.data + 3;
  const real_t *q_j = pose_j.data + 3;
  const real_t dt = ts2sec(ts_j) - ts2sec(ts_i);
  const real_t *a_i = imu_buf.acc[k - 1];
  const real_t *w_i = imu_buf.gyr[k - 1];
  const real_t *a_j = imu_buf.acc[k];
  const real_t *w_j = imu_buf.gyr[k];
  real_t F_dt[15 * 15] = {0};
  imu_factor_F_matrix(q_i, q_j, ba_i, bg_i, a_i, w_i, a_j, w_j, dt, F_dt);
  // mat_save("/tmp/F.csv", F_dt, 15, 15);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_factor(void) {
  // Setup test data
  const double circle_r = 1.0;
  const double circle_v = 0.1;
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, circle_r, circle_v);

  // Setup IMU buffer
  int buf_size = 100;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < buf_size; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = buf_size - 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0.0, 0.0, 0.0};
  const real_t bg_i[3] = {0.0, 0.0, 0.0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0.0, 0.0, 0.0};
  const real_t bg_j[3] = {0.0, 0.0, 0.0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  imu_params_t imu_params;
  imu_params.imu_idx = 0;
  imu_params.rate = 200.0;
  imu_params.sigma_a = 0.08;
  imu_params.sigma_g = 0.004;
  imu_params.sigma_aw = 0.00004;
  imu_params.sigma_gw = 2.0e-6;
  imu_params.g = 9.81;

  pose_j.data[0] += 0.01;
  pose_j.data[1] += 0.02;
  pose_j.data[2] += 0.03;

  imu_factor_t factor;
  imu_factor_setup(&factor,
                   &imu_params,
                   &imu_buf,
                   &pose_i,
                   &vel_i,
                   &biases_i,
                   &pose_j,
                   &vel_j,
                   &biases_j);
  imu_factor_eval(&factor);

  // print_vector("pose_i", pose_i.data, 7);
  // print_vector("pose_j", pose_j.data, 7);
  // print_vector("vel_i", vel_i.data, 3);
  // print_vector("vel_j", vel_j.data, 3);
  // print_vector("dr", factor.dr, 3);
  // print_vector("dv", factor.dv, 3);
  // print_vector("dq", factor.dq, 4);
  // printf("dt: %f\n", factor.Dt);
  // print_vector("r", factor.r, 15);
  // mat_save("/tmp/F.csv", factor.F, 15, 15);
  // mat_save("/tmp/P.csv", factor.P, 15, 15);
  // mat_save("/tmp/r.csv", factor.r, 15, 1);
  // mat_save("/tmp/pose_i.csv", factor.params[0], 7, 1);
  // mat_save("/tmp/vel_i.csv", factor.params[1], 3, 1);
  // mat_save("/tmp/pose_j.csv", factor.params[3], 7, 1);
  // mat_save("/tmp/vel_j.csv", factor.params[4], 3, 1);

  for (int i =0; i < 15; i++) {
    printf("%.4e\n", factor.r[i]);
  }
  printf("\n");

  /*
  const char *cmd = "\
F = csvread('/tmp/F.csv'); \
state_F = csvread('/tmp/F_.csv'); \
subplot(311); \
imagesc(F); \
title('Python'); \
axis 'equal'; \
colorbar(); \
subplot(312); \
imagesc(state_F); \
title('C'); \
axis 'equal'; \
colorbar(); \
subplot(313); \
imagesc(F - state_F); \
title('F - stateF'); \
colorbar(); \
axis 'equal'; \
F(1:3, 1:3); \
state_F(1:3, 1:3); \
ginput();\
";
  char syscmd[9046] = {0};
  sprintf(syscmd, "octave-cli --eval \"%s\"", cmd);
  system(syscmd);
  */

  // MU_ASSERT(factor.pose_i == &pose_i);
  // MU_ASSERT(factor.vel_i == &vel_i);
  // MU_ASSERT(factor.biases_i == &biases_i);
  // MU_ASSERT(factor.pose_i == &pose_i);
  // MU_ASSERT(factor.vel_j == &vel_j);
  // MU_ASSERT(factor.biases_j == &biases_j);
  //
  // // Check Jacobians
  // const double tol = 1e-4;
  // const double step_size = 1e-8;
  // eye(factor.sqrt_info, 15, 15);
  // CHECK_FACTOR_J(0, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(1, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(2, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(3, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(4, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(5, factor, imu_factor_eval, step_size, tol, 0);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_joint_factor(void) {
  // Joint angle
  const timestamp_t ts = 0;
  const int joint_idx = 0;
  const real_t z = 0.01;
  joint_t joint;
  joint_setup(&joint, ts, joint_idx, z);

  // Joint angle factor
  joint_factor_t factor;
  const real_t var = 0.1;
  joint_factor_setup(&factor, &joint, z, var);

  // Evaluate
  joint_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  CHECK_FACTOR_J(0, factor, joint_factor_eval, step_size, tol, 0);

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

// static void setup_calib_gimbal_factor(calib_gimbal_factor_t *factor,
//                                       fiducial_t *fiducial_ext,
//                                       extrinsic_t *gimbal_ext,
//                                       pose_t *pose,
//                                       extrinsic_t *link0,
//                                       extrinsic_t *link1,
//                                       joint_t *joint0,
//                                       joint_t *joint1,
//                                       joint_t *joint2,
//                                       extrinsic_t *cam_exts,
//                                       camera_params_t *cam) {
//   // Body pose T_WB
//   real_t ypr_WB[3] = {0.0, 0.0, 0.0};
//   real_t r_WB[3] = {0.0, 0.0, 0.0};
//   real_t T_WB[4 * 4] = {0};
//   tf_er(ypr_WB, r_WB, T_WB);
//
//   real_t x_WB[7] = {0};
//   tf_vector(T_WB, x_WB);
//   pose_setup(pose, 0, x_WB);
//
//   // Fiducial pose T_WF
//   real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
//   real_t r_WF[3] = {0.5, 0.0, 0.0};
//   real_t T_WF[4 * 4] = {0};
//   tf_er(ypr_WF, r_WF, T_WF);
//
//   real_t x_WF[7] = {0};
//   tf_vector(T_WF, x_WF);
//   fiducial_setup(fiducial_ext, x_WF);
//
//   // Relative fiducial pose T_BF
//   real_t T_BF[4 * 4] = {0};
//   TF_INV(T_WB, T_BW);
//   dot(T_BW, 4, 4, T_WF, 4, 4, T_BF);
//
//   // Gimbal extrinsic
//   real_t ypr_BM0[3] = {0.01, 0.01, 0.01};
//   real_t r_BM0[3] = {0.0, 0.0, 0.0};
//   real_t T_BM0[4 * 4] = {0};
//   gimbal_setup_extrinsic(ypr_BM0, r_BM0, T_BM0, gimbal_ext);
//
//   // Roll link
//   real_t ypr_L0M1[3] = {0.0, M_PI / 2, 0.0};
//   real_t r_L0M1[3] = {-0.1, 0.0, 0.15};
//   real_t T_L0M1[4 * 4] = {0};
//   gimbal_setup_extrinsic(ypr_L0M1, r_L0M1, T_L0M1, link0);
//
//   // Pitch link
//   real_t ypr_L1M2[3] = {0.0, 0.0, -M_PI / 2.0};
//   real_t r_L1M2[3] = {0.0, -0.05, 0.1};
//   real_t T_L1M2[4 * 4] = {0};
//   gimbal_setup_extrinsic(ypr_L1M2, r_L1M2, T_L1M2, link1);
//
//   // Joint0
//   const real_t th0 = 0.01;
//   real_t T_M0L0[4 * 4] = {0};
//   gimbal_setup_joint(0, 0, th0, T_M0L0, joint0);
//
//   // Joint1
//   const real_t th1 = 0.02;
//   real_t T_M1L1[4 * 4] = {0};
//   gimbal_setup_joint(0, 1, th1, T_M1L1, joint1);
//
//   // Joint2
//   const real_t th2 = 0.03;
//   real_t T_M2L2[4 * 4] = {0};
//   gimbal_setup_joint(0, 2, th2, T_M2L2, joint2);
//
//   // Camera extrinsic
//   const real_t ypr_L2C0[3] = {-M_PI / 2, M_PI / 2, 0.0};
//   const real_t r_L2C0[3] = {0.0, -0.05, 0.12};
//   real_t T_L2C0[4 * 4] = {0};
//   gimbal_setup_extrinsic(ypr_L2C0, r_L2C0, T_L2C0, cam_exts);
//
//   // Camera parameters K
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t fov = 120.0;
//   const real_t fx = pinhole_focal(cam_res[0], fov);
//   const real_t fy = pinhole_focal(cam_res[0], fov);
//   const real_t cx = cam_res[0] / 2.0;
//   const real_t cy = cam_res[1] / 2.0;
//   const real_t cam_params[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
//   camera_params_setup(cam,
//                       cam_idx,
//                       cam_res,
//                       proj_model,
//                       dist_model,
//                       cam_params);
//
//   // Form T_C0F
//   real_t T_BC0[4 * 4] = {0};
//   real_t T_C0F[4 * 4] = {0};
//   tf_chain2(7, T_BM0, T_M0L0, T_L0M1, T_M1L1, T_L1M2, T_M2L2, T_L2C0, T_BC0);
//   TF_INV(T_BC0, T_C0B);
//   tf_chain2(2, T_C0B, T_BF, T_C0F);
//
//   // Project point to image plane
//   const real_t p_FFi[3] = {0.0, 0.0, 0.0};
//   real_t p_C0Fi[3] = {0};
//   real_t z[2] = {0};
//   tf_point(T_C0F, p_FFi, p_C0Fi);
//   pinhole_radtan4_project(cam_params, p_C0Fi, z);
//
//   // Setup factor
//   const timestamp_t ts = 0;
//   const int tag_id = 0;
//   const int corner_idx = 0;
//   const real_t var[2] = {1.0, 1.0};
//   calib_gimbal_factor_setup(factor,
//                             fiducial_ext,
//                             gimbal_ext,
//                             pose,
//                             link0,
//                             link1,
//                             joint0,
//                             joint1,
//                             joint2,
//                             cam_exts,
//                             cam,
//                             ts,
//                             cam_idx,
//                             tag_id,
//                             corner_idx,
//                             p_FFi,
//                             z,
//                             var);
// }
//
// int test_calib_gimbal_factor(void) {
//   calib_gimbal_factor_t factor;
//   fiducial_t fiducial_ext;
//   extrinsic_t gimbal_ext;
//   pose_t pose;
//   extrinsic_t link0;
//   extrinsic_t link1;
//   joint_t joint0;
//   joint_t joint1;
//   joint_t joint2;
//   extrinsic_t cam_exts;
//   camera_params_t cam;
//   setup_calib_gimbal_factor(&factor,
//                             &fiducial_ext,
//                             &gimbal_ext,
//                             &pose,
//                             &link0,
//                             &link1,
//                             &joint0,
//                             &joint1,
//                             &joint2,
//                             &cam_exts,
//                             &cam);
//
//   // Evaluate
//   calib_gimbal_factor_eval(&factor);
//
//   // Check Jacobians
//   const double tol = 1e-4;
//   const double step_size = 1e-8;
//   CHECK_FACTOR_J(0, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(1, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(2, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(3, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(4, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(5, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(6, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(7, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//   CHECK_FACTOR_J(8, factor, calib_gimbal_factor_eval, step_size, tol, 0);
//
//   return 0;
// }

int test_marg(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Extrinsic T_BC
  extrinsic_t cam_ext;
  const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsic_setup(&cam_ext, ext_data);
  cam_ext.fix = 1;

  // Camera parameters
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Setup features and poses
  int num_poses = 5;
  int num_features = 10;
  pose_t poses[20] = {0};
  feature_t features[100] = {0};
  real_t points[100 * 3] = {0};
  real_t keypoints[100 * 2] = {0};
  camera_factor_t factors[20 * 100];

  for (int i = 0; i < num_features; i++) {
    const real_t dx = randf(-0.5, 0.5);
    const real_t dy = randf(-0.5, 0.5);
    const real_t dz = randf(-0.5, 0.5);
    const real_t p_W[3] = {3.0 + dx, 0.0 + dy, 0.0 + dz};
    feature_t *feature = &features[i];
    feature_init(feature, 0, p_W);
    points[i * 3 + 0] = p_W[0];
    points[i * 3 + 1] = p_W[1];
    points[i * 3 + 2] = p_W[2];
  }

  int factor_idx = 0;
  for (int k = 0; k < num_poses; k++) {
    // Body pose T_WB
    const real_t dx = randf(-0.05, 0.05);
    const real_t dy = randf(-0.05, 0.05);
    const real_t dz = randf(-0.05, 0.05);

    const real_t droll = randf(-0.2, 0.2);
    const real_t dpitch = randf(-0.2, 0.2);
    const real_t dyaw = randf(-0.1, 0.1);
    const real_t ypr[3] = {dyaw, dpitch, droll};
    real_t q[4] = {0};
    euler2quat(ypr, q);

    pose_t *pose = &poses[k];
    real_t pose_data[7] = {dx, dy, dz, q[0], q[1], q[2], q[3]};
    pose_setup(pose, ts + k, pose_data);
    pose->marginalize = (k == 0) ? 1 : 0; // Marginalize 1st pose

    for (int i = 0; i < num_features; i++) {
      // Project point from world to image plane
      real_t *p_W = &points[i * 3];
      TF(pose_data, T_WB);
      TF(ext_data, T_BCi);
      TF_INV(T_BCi, T_CiB);
      TF_INV(T_WB, T_BW);
      DOT(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
      TF_POINT(T_CiW, p_W, p_Ci);

      real_t z[2];
      pinhole_radtan4_project(cam_data, p_Ci, z);
      keypoints[i * 2 + 0] = z[0] + 0.001;
      keypoints[i * 2 + 1] = z[1] - 0.001;

      // Setup camera factor
      camera_factor_t *cam_factor = &factors[factor_idx];
      feature_t *feature = &features[i];
      real_t var[2] = {1.0, 1.0};
      camera_factor_setup(cam_factor, pose, &cam_ext, feature, &cam, z, var);
      camera_factor_eval(cam_factor);
      factor_idx++;
    }
  }
  UNUSED(keypoints);

  // Determine parameter order
  param_order_t *hash = NULL;
  int col_idx = 0;
  // -- Add body poses
  for (int i = 0; i < num_poses; i++) {
    void *data = poses[i].data;
    const int fix = 0;
    param_order_add(&hash, POSE_PARAM, fix, data, &col_idx);
  }
  // -- Add points
  for (int i = 0; i < num_features; i++) {
    void *data = &features[i].data;
    const int fix = 0;
    param_order_add(&hash, FEATURE_PARAM, fix, data, &col_idx);
  }
  // -- Add camera extrinsic
  {
    void *data = cam_ext.data;
    const int fix = 1;
    param_order_add(&hash, EXTRINSIC_PARAM, fix, data, &col_idx);
  }
  // -- Add camera parameters
  {
    void *data = cam.data;
    const int fix = 0;
    param_order_add(&hash, CAMERA_PARAM, fix, data, &col_idx);
  }
  // -- Misc
  const int sv_size = col_idx;
  const int r_size = (factor_idx * 2);

  // Form Hessian **before** marginalization
  int r_idx = 0;
  real_t *H = CALLOC(real_t, sv_size * sv_size);
  real_t *g = CALLOC(real_t, sv_size * 1);
  real_t *r = CALLOC(real_t, r_size * 1);
  for (int i = 0; i < (num_poses * num_features); i++) {
    camera_factor_t *factor = &factors[i];
    camera_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[r_idx]);

    solver_fill_hessian(hash,
                        factor->num_params,
                        factor->params,
                        factor->jacs,
                        factor->r,
                        factor->r_size,
                        sv_size,
                        H,
                        g);
    r_idx += factor->r_size;
  }

  // Setup marginalization factor
  marg_factor_t *marg = marg_factor_malloc();
  for (int i = 0; i < (num_poses * num_features); i++) {
    marg_factor_add(marg, CAMERA_FACTOR, &factors[i]);
  }
  marg_factor_marginalize(marg);
  // marg_factor_eval(marg);

  // Print timings
  // printf("marg->time_hessian_form:     %.4fs\n", marg->time_hessian_form);
  // printf("marg->time_schur_complement: %.4fs\n", marg->time_schur_complement);
  // printf("marg->time_hessian_decomp:   %.4fs\n", marg->time_hessian_decomp);
  // printf("marg->time_fejs:             %.4fs\n", marg->time_fejs);
  // printf("------------------------------------\n");
  // printf("marg->time_total:            %.4fs\n", marg->time_total);

  // Determine parameter order for the marginalized Hessian
  param_order_t *hash_ = NULL;
  int col_idx_ = 0;
  // -- Add body poses
  for (int i = 0; i < num_poses; i++) {
    void *data = poses[i].data;
    const int fix = poses[i].marginalize;
    param_order_add(&hash_, POSE_PARAM, fix, data, &col_idx_);
  }
  // -- Add points
  for (int i = 0; i < num_features; i++) {
    void *data = &features[i].data;
    const int fix = 0;
    param_order_add(&hash_, FEATURE_PARAM, fix, data, &col_idx_);
  }
  // -- Add camera extrinsic
  {
    void *data = cam_ext.data;
    const int fix = 1;
    param_order_add(&hash_, EXTRINSIC_PARAM, fix, data, &col_idx_);
  }
  // -- Add camera parameters
  {
    void *data = cam.data;
    const int fix = 0;
    param_order_add(&hash_, CAMERA_PARAM, fix, data, &col_idx_);
  }
  // -- Misc
  const int sv_size_ = col_idx_;

  // Form marg hessian
  real_t *H_ = CALLOC(real_t, sv_size_ * sv_size_);
  real_t *g_ = CALLOC(real_t, sv_size_ * 1);
  solver_fill_hessian(hash_,
                      marg->num_params,
                      marg->params,
                      marg->jacs,
                      marg->r,
                      marg->r_size,
                      sv_size_,
                      H_,
                      g_);

  // Clean up
  marg_factor_free(marg);
  param_order_free(hash);
  free(H);
  free(g);
  free(r);

  param_order_free(hash_);
  free(H_);
  free(g_);

  return 0;
}

int test_visual_odometry_batch(void) {
  // Simulate features
  const real_t origin[3] = {0.0, 0.0, 0.0};
  const real_t dim[3] = {5.0, 5.0, 5.0};
  const int num_features = 1000;
  real_t features[3 * 1000] = {0};
  sim_create_features(origin, dim, num_features, features);

  // Camera configuration
  const int res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(res[0], fov);
  const real_t fy = pinhole_focal(res[0], fov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  camera_params_t cam0_params;
  camera_params_setup(&cam0_params, 0, res, pmodel, dmodel, cam_vec);

  // IMU-Camera0 extrinsic
  const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
  TF_ER(cam0_ext_ypr, cam0_ext_r, T_BC0);
  TF_VECTOR(T_BC0, cam0_ext);
  TF_INV(T_BC0, T_C0B);

  // Simulate data
  sim_circle_t conf;
  sim_circle_defaults(&conf);
  sim_camera_data_t *cam0_data = sim_camera_circle_trajectory(&conf,
                                                              T_BC0,
                                                              &cam0_params,
                                                              features,
                                                              num_features);
  // Setup factor graph
  // feature_hash_t *feature_params = NULL;

  // fgraph_t *fg = fgraph_malloc();
  // // -- Add features
  // for (int feature_id = 0; feature_id < num_features; feature_id++) {
  //   fgraph_add_feature(fg, feature_id, features + feature_id * 3, 0);
  // }
  // // -- Add camera
  // const int cam0_id = fgraph_add_camera(fg, 0, res, pmodel, dmodel, cam_vec, 0);
  // const int cam0_ext_id = fgraph_add_cam_ext(fg, 0, cam0_ext, 0);

  // for (size_t k = 0; k < cam0_data->num_frames; k++) {
  //   const sim_camera_frame_t *cam0_frame = cam0_data->frames[k];

  //   // Add pose
  //   const real_t *cam0_pose = &cam0_data->poses[k * 7];
  //   TF(cam0_pose, T_WC0);
  //   TF_CHAIN(T_WB, 2, T_WC0, T_C0B);
  //   TF_VECTOR(T_WB, body_pose);
  //   pose_random_perturb(body_pose, 0.1, 0.1);
  //   const int pose_id = fgraph_add_pose(fg, k, body_pose, 0);

  //   // Add camera factors
  //   for (int i = 0; i < cam0_frame->num_measurements; i++) {
  //     const int feature_id = cam0_frame->feature_ids[i];
  //     const real_t *kp = cam0_frame->keypoints + i * 2;
  //     const int param_ids[4] = {pose_id, cam0_ext_id, feature_id, cam0_id};
  //     const real_t var[2] = {1.0, 1.0};
  //     fgraph_add_camera_factor(fg, param_ids, kp, var);
  //   }
  // }

  // // Solve
  // solver_t solver;
  // solver_setup(&solver);

  // solver.verbose = 1;
  // solver.max_iter = 5;
  // solver.cost_func = &fgraph_cost;
  // solver.param_order_func = &fgraph_param_order;
  // solver.linearize_func = &fgraph_linearize_compact;
  // solver_solve(&solver, fg);

  // Clean up
  sim_camera_data_free(cam0_data);
  // fgraph_free(fg);

  return 0;
}

int test_inertial_odometry_batch(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Inertial Odometry
  const int num_partitions = test_data.num_measurements / 20.0;
  const size_t N = test_data.num_measurements / (real_t) num_partitions;
  inertial_odometry_t *odom = MALLOC(inertial_odometry_t, 1);
  // -- IMU params
  odom->imu_params.imu_idx = 0;
  odom->imu_params.rate = 200.0;
  odom->imu_params.sigma_a = 0.08;
  odom->imu_params.sigma_g = 0.004;
  odom->imu_params.sigma_aw = 0.00004;
  odom->imu_params.sigma_gw = 2.0e-6;
  odom->imu_params.g = 9.81;
  // -- Variables
  odom->num_factors = 0;
  odom->factors = MALLOC(imu_factor_t, num_partitions);
  odom->poses = MALLOC(pose_t, num_partitions + 1);
  odom->vels = MALLOC(velocity_t, num_partitions + 1);
  odom->biases = MALLOC(imu_biases_t, num_partitions + 1);

  const timestamp_t ts_i = test_data.timestamps[0];
  const real_t *v_i = test_data.velocities[0];
  const real_t ba_i[3] = {0, 0, 0};
  const real_t bg_i[3] = {0, 0, 0};
  pose_setup(&odom->poses[0], ts_i, test_data.poses[0]);
  velocity_setup(&odom->vels[0], ts_i, v_i);
  imu_biases_setup(&odom->biases[0], ts_i, ba_i, bg_i);

  for (int i = 1; i < num_partitions; i++) {
    const int ks = i * N;
    const int ke = MIN((i + 1) * N - 1, test_data.num_measurements - 1);

    // Setup imu buffer
    imu_buffer_t imu_buf;
    imu_buffer_setup(&imu_buf);
    for (size_t k = 0; k < N; k++) {
      const timestamp_t ts = test_data.timestamps[ks + k];
      const real_t *acc = test_data.imu_acc[ks + k];
      const real_t *gyr = test_data.imu_gyr[ks + k];
      imu_buffer_add(&imu_buf, ts, acc, gyr);
    }

    // Setup parameters
    const timestamp_t ts_j = test_data.timestamps[ke];
    const real_t *v_j = test_data.velocities[ke];
    const real_t ba_j[3] = {0, 0, 0};
    const real_t bg_j[3] = {0, 0, 0};
    pose_setup(&odom->poses[i], ts_j, test_data.poses[ke]);
    velocity_setup(&odom->vels[i], ts_j, v_j);
    imu_biases_setup(&odom->biases[i], ts_j, ba_j, bg_j);

    // Setup IMU factor
    imu_factor_setup(&odom->factors[i - 1],
                     &odom->imu_params,
                     &imu_buf,
                     &odom->poses[i - 1],
                     &odom->vels[i - 1],
                     &odom->biases[i - 1],
                     &odom->poses[i],
                     &odom->vels[i],
                     &odom->biases[i]);
    odom->num_factors++;
  }

  // Save ground truth
  inertial_odometry_save(odom, "/tmp/imu_odom-gnd.csv");

  // Perturb ground truth
  for (int k = 0; k <= odom->num_factors; k++) {
    odom->poses[k].data[0] += randf(-1.0, 1.0);
    odom->poses[k].data[1] += randf(-1.0, 1.0);
    odom->poses[k].data[2] += randf(-1.0, 1.0);
    quat_perturb(odom->poses[k].data + 3, 0, randf(-1e-1, 1e-1));
    quat_perturb(odom->poses[k].data + 3, 1, randf(-1e-1, 1e-1));
    quat_perturb(odom->poses[k].data + 3, 2, randf(-1e-1, 1e-1));

    odom->vels[k].data[0] += randf(-1.0, 1.0);
    odom->vels[k].data[1] += randf(-1.0, 1.0);
    odom->vels[k].data[2] += randf(-1.0, 1.0);
  }
  inertial_odometry_save(odom, "/tmp/imu_odom-init.csv");

  // Solve
  solver_t solver;
  solver_setup(&solver);
  solver.verbose = 1;
  solver.param_order_func = &inertial_odometry_param_order;
  solver.cost_func = &inertial_odometry_cost;
  solver.linearize_func = &inertial_odometry_linearize_compact;
  solver_solve(&solver, odom);
  inertial_odometry_save(odom, "/tmp/imu_odom-est.csv");

  // Clean up
  inertial_odometry_free(odom);
  free_imu_test_data(&test_data);

  return 0;
}

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

int test_tsf(void) {
  // Camera config
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";

  const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
  const real_t cam1_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam1_ext_r[3] = {-0.05, 0.0, 0.0};
  TF_ER(cam0_ext_ypr, cam0_ext_r, T_SC0);
  TF_ER(cam1_ext_ypr, cam1_ext_r, T_SC1);
  TF_VECTOR(T_SC0, cam0_ext);
  TF_VECTOR(T_SC1, cam1_ext);

  // IMU config
  const real_t imu_rate = 200;
  const real_t sigma_a = 0.08;
  const real_t sigma_g = 0.004;
  const real_t sigma_aw = 0.00004;
  const real_t sigma_gw = 2.0e-6;
  const real_t g = 9.81;
  TF_IDENTITY(T_BS);
  TF_VECTOR(T_BS, imu0_ext);

  // Simulate VO
  tsf_t *tsf = tsf_malloc();
  tsf_add_camera(tsf, 0, cam_res, pmodel, dmodel, cam_vec, cam0_ext);
  tsf_add_camera(tsf, 1, cam_res, pmodel, dmodel, cam_vec, cam1_ext);
  tsf_add_imu(tsf, imu_rate, sigma_aw, sigma_gw, sigma_a, sigma_g, g, imu0_ext);

  // Loop through timeline
  sim_circle_camera_imu_t *sim_data = sim_circle_camera_imu();
  timeline_t *timeline = sim_data->timeline;
  tsf_set_init_pose(tsf, &sim_data->imu_data->poses[0]);
  tsf_set_init_velocity(tsf, &sim_data->imu_data->velocities[0]);

  for (int k = 0; k < timeline->timeline_length; k++) {
    const timestamp_t ts = timeline->timeline_events[k][0]->ts;

    // Extract timeline events
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        tsf_imu_event(tsf, ts, data->acc, data->gyr);
      }
    }

    tsf_update(tsf, ts);
  }

  // Clean up
  sim_circle_camera_imu_free(sim_data);
  tsf_free(tsf);

  return 0;
}

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

// int test_solver_eval(void) {
//   // Load test data
//   const char *dir_path = TEST_SIM_DATA "/cam0";
//   sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t params[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, params);

//   // Setup features
//   // -- Load features csv
//   int num_rows = 0;
//   int num_cols = 0;
//   char *features_csv = TEST_SIM_DATA "/features.csv";
//   real_t **features_data = csv_data(features_csv, &num_rows, &num_cols);
//   size_t *feature_ids = MALLOC(size_t, num_rows);
//   real_t *feature_xyzs = MALLOC(real_t, num_rows * 3);
//   for (int i = 0; i < num_rows; i++) {
//     feature_ids[i] = features_data[i][0];
//     feature_xyzs[i * 3 + 0] = features_data[i][1];
//     feature_xyzs[i * 3 + 1] = features_data[i][2];
//     feature_xyzs[i * 3 + 2] = features_data[i][3];
//     free(features_data[i]);
//   }
//   free(features_data);
//   // -- Add features to container
//   features_t *features = features_malloc();
//   features_add_xyzs(features, feature_ids, feature_xyzs, num_rows);
//   free(feature_ids);
//   free(feature_xyzs);

//   // Loop over simulated camera frames
//   const real_t var[2] = {1.0, 1.0};
//   cam_view_t *cam_views = MALLOC(cam_view_t, cam_data->num_frames);
//   for (int k = 0; k < cam_data->num_frames; k++) {
//     // Camera frame
//     const sim_camera_frame_t *frame = cam_data->frames[k];

//     // Pose
//     pose_t *pose = &cam_views[k].pose;
//     pose_setup(pose, frame->ts, &cam_data->poses[k]);

//     // Add factors
//     cam_views[k].num_factors = frame->num_measurements;
//     for (int i = 0; i < frame->num_measurements; i++) {
//       const int feature_id = frame->feature_ids[i];
//       const real_t *z = &frame->keypoints[i];
//       feature_t *f = NULL;
//       features_get_xyz(features, feature_id, &f);

//       // Factor
//       ba_factor_t *factor = &cam_views[k].factors[i];
//       ba_factor_setup(factor, pose, f, &cam, z, var);
//     }
//   }

//   solver_t solver;
//   solver_setup(&solver);

//   // Clean up
//   sim_camera_data_free(cam_data);
//   free(cam_views);
//   features_free(features);

//   return 0;
// }

// int test_camchain(void) {
//   // Form camera poses
//   int num_cams = 5;
//   real_t T_C0F[4 * 4] = {0};
//   real_t T_C1F[4 * 4] = {0};
//   real_t T_C2F[4 * 4] = {0};
//   real_t T_C3F[4 * 4] = {0};
//   real_t T_C4F[4 * 4] = {0};
//   real_t *poses[5] = {T_C0F, T_C1F, T_C2F, T_C3F, T_C4F};
//
//   for (int k = 0; k < num_cams; k++) {
//     const real_t ypr[3] = {randf(-90, 90), randf(-90, 90), randf(-90, 90)};
//     const real_t r[3] = {randf(-1, 1), randf(-1, 1), randf(-1, 1)};
//     tf_er(ypr, r, poses[k]);
//   }
//
//   TF_INV(T_C0F, T_FC0);
//   TF_INV(T_C1F, T_FC1);
//   TF_INV(T_C2F, T_FC2);
//   TF_INV(T_C3F, T_FC3);
//   TF_INV(T_C4F, T_FC4);
//   real_t *poses_inv[5] = {T_FC0, T_FC1, T_FC2, T_FC3, T_FC4};
//
//   // Camchain
//   camchain_t *camchain = camchain_malloc(num_cams);
//   camchain_add_pose(camchain, 0, 0, T_C0F);
//   camchain_add_pose(camchain, 1, 0, T_C1F);
//   camchain_add_pose(camchain, 2, 0, T_C2F);
//   camchain_add_pose(camchain, 3, 0, T_C3F);
//   camchain_adjacency(camchain);
//   // camchain_adjacency_print(camchain);
//
//   for (int cam_i = 1; cam_i < num_cams; cam_i++) {
//     for (int cam_j = 1; cam_j < num_cams; cam_j++) {
//       // Get ground-truth
//       TF_CHAIN(T_CiCj_gnd, 2, poses[cam_i], poses_inv[cam_j]);
//
//       // Get camchain result
//       real_t T_CiCj_est[4 * 4] = {0};
//       int status = camchain_find(camchain, cam_i, cam_j, T_CiCj_est);
//
//       if (cam_i != 4 && cam_j != 4) { // Camera 4 was not added
//         MU_ASSERT(status == 0);
//       } else {
//         MU_ASSERT(status == -1);
//       }
//     }
//   }
//
//   // Clean up
//   camchain_free(camchain);
//
//   return 0;
// }
//
// int test_calib_camera_mono_batch(void) {
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
//   calib->max_iter = 30;
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
//   calib_camera_solve(calib);
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
//
//   return 0;
// }
//
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
  // SENSOR FUSION
  MU_ADD_TEST(test_schur_complement);
  // MU_ADD_TEST(test_timeline);
  MU_ADD_TEST(test_pose);
  MU_ADD_TEST(test_extrinsics);
  MU_ADD_TEST(test_fiducial);
  // MU_ADD_TEST(test_fiducial_buffer);
  MU_ADD_TEST(test_imu_biases);
  MU_ADD_TEST(test_feature);
  // MU_ADD_TEST(test_features);
  // MU_ADD_TEST(test_idf);
  // MU_ADD_TEST(test_idfb);
  MU_ADD_TEST(test_time_delay);
  MU_ADD_TEST(test_joint);
  MU_ADD_TEST(test_camera_params);
  MU_ADD_TEST(test_triangulation_batch);
  MU_ADD_TEST(test_pose_factor);
  MU_ADD_TEST(test_ba_factor);
  MU_ADD_TEST(test_camera_factor);
  // MU_ADD_TEST(test_idf_factor);
  MU_ADD_TEST(test_imu_buffer_setup);
  MU_ADD_TEST(test_imu_buffer_add);
  MU_ADD_TEST(test_imu_buffer_clear);
  MU_ADD_TEST(test_imu_buffer_copy);
  MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_imu_initial_attitude);
  MU_ADD_TEST(test_imu_factor_form_F_matrix);
  MU_ADD_TEST(test_imu_factor);
  MU_ADD_TEST(test_joint_factor);
  MU_ADD_TEST(test_calib_camera_factor);
  MU_ADD_TEST(test_calib_imucam_factor);
  // MU_ADD_TEST(test_calib_gimbal_factor);
  MU_ADD_TEST(test_marg);
  // MU_ADD_TEST(test_visual_odometry_batch);
  MU_ADD_TEST(test_inertial_odometry_batch);
  // MU_ADD_TEST(test_visual_inertial_odometry_batch);
  // MU_ADD_TEST(test_tsf);
#ifdef USE_CERES
  MU_ADD_TEST(test_ceres_example);
#endif // USE_CERES
  MU_ADD_TEST(test_solver_setup);
  // MU_ADD_TEST(test_solver_eval);
  // MU_ADD_TEST(test_camchain);
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
  // MU_ADD_TEST(test_calib_gimbal_copy);
  // MU_ADD_TEST(test_calib_gimbal_add_fiducial);
  // MU_ADD_TEST(test_calib_gimbal_add_pose);
  // MU_ADD_TEST(test_calib_gimbal_add_gimbal_extrinsic);
  // MU_ADD_TEST(test_calib_gimbal_add_gimbal_link);
  // MU_ADD_TEST(test_calib_gimbal_add_camera);
  // MU_ADD_TEST(test_calib_gimbal_add_remove_view);
  // MU_ADD_TEST(test_calib_gimbal_load);
  // MU_ADD_TEST(test_calib_gimbal_save);
  // MU_ADD_TEST(test_calib_gimbal_solve);
#ifdef USE_CERES
  // MU_ADD_TEST(test_calib_gimbal_ceres_solve);
#endif // USE_CERES
}
MU_RUN_TESTS(test_suite)
