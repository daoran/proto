#include "munit.h"
#include "xyz.h"
#include "xyz_se.h"
#include "xyz_calib.h"
#include "xyz_aprilgrid.h"
#include "xyz_sim.h"
#include "xyz_gui.h"
#include "xyz_kitti.h"

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

  // real_t *Hmm = malloc(sizeof(real_t) * m * m);
  // real_t *Hmr = malloc(sizeof(real_t) * m * r);
  // real_t *Hrm = malloc(sizeof(real_t) * m * r);
  // real_t *Hrr = malloc(sizeof(real_t) * r * r);
  // real_t *Hmm_inv = malloc(sizeof(real_t) * m * m);

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
//   size_t *idf_ids = malloc(sizeof(size_t) * num_idfs);
//   real_t *idf_features = malloc(sizeof(real_t) * num_idfs * 3);
//   real_t *idf_keypoints = malloc(sizeof(real_t) * num_idfs * 2);
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
  real_t *kps_i = malloc(sizeof(real_t) * N * 2);
  real_t *kps_j = malloc(sizeof(real_t) * N * 2);
  real_t *points_gnd = malloc(sizeof(real_t) * N * 3);
  real_t *points_est = malloc(sizeof(real_t) * N * 3);
  int *status = malloc(sizeof(int) * N);

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
  test_data->timestamps = calloc(test_data->num_measurements, sizeof(real_t));
  test_data->poses = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->velocities = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->imu_acc = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->imu_gyr = calloc(test_data->num_measurements, sizeof(real_t *));

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

  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_i == &vel_i);
  MU_ASSERT(factor.biases_i == &biases_i);
  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_j == &vel_j);
  MU_ASSERT(factor.biases_j == &biases_j);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  eye(factor.sqrt_info, 15, 15);
  CHECK_FACTOR_J(0, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(3, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(4, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(5, factor, imu_factor_eval, step_size, tol, 0);

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
  real_t *H = calloc(sv_size * sv_size, sizeof(real_t));
  real_t *g = calloc(sv_size, sizeof(real_t));
  real_t *r = calloc(r_size, sizeof(real_t));
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
  marg_factor_eval(marg);

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
  real_t *H_ = calloc(sv_size_ * sv_size_, sizeof(real_t));
  real_t *g_ = calloc(sv_size_ * 1, sizeof(real_t));
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

  return 0;
}

int test_inertial_odometry_batch(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Inertial Odometry
  const int num_partitions = test_data.num_measurements / 20.0;
  const size_t N = test_data.num_measurements / (real_t) num_partitions;
  inertial_odometry_t *odom = malloc(sizeof(inertial_odometry_t) * 1);
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
  odom->factors = malloc(sizeof(imu_factor_t) * num_partitions);
  odom->poses = malloc(sizeof(pose_t) * num_partitions + 1);
  odom->vels = malloc(sizeof(velocity_t) * num_partitions + 1);
  odom->biases = malloc(sizeof(imu_biases_t) * num_partitions + 1);

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

void vis_kitti_scan(gl_points3d_t *gl_points,
                    const gl_float_t *pcd,
                    const size_t num_points) {
  gl_float_t point_size = 2.0;
  gl_float_t *points_data = malloc(sizeof(gl_float_t) * num_points * 6);

  for (size_t i = 0; i < num_points; ++i) {
    // Point positions
    points_data[i * 6 + 0] = pcd[i * 4 + 1];
    points_data[i * 6 + 1] = pcd[i * 4 + 2] + 1.6;
    points_data[i * 6 + 2] = -pcd[i * 4 + 0];

    // Point color
    gl_float_t r = 0.0f;
    gl_float_t g = 0.0f;
    gl_float_t b = 0.0f;
    gl_jet_colormap((pcd[i * 4 + 2] + 1.6) / 3.0, &r, &g, &b);
    points_data[i * 6 + 3] = r;
    points_data[i * 6 + 4] = g;
    points_data[i * 6 + 5] = b;
  }
  gl_points3d_update(gl_points, points_data, num_points, point_size);
  free(points_data);
}

int test_icp(void) {
  // Setup
  const char *window_title = "viz";
  const int window_width = 1024;
  const int window_height = 768;
  gui_t *gui = gui_malloc(window_title, window_width, window_height);

  // Grid
  gl_float_t grid_size = 0.5f;
  gl_float_t grid_lw = 5.0f;
  gl_color_t grid_color = (gl_color_t){0.9, 0.4, 0.2};
  gl_grid3d_t *gl_grid = gl_grid3d_malloc(grid_size, grid_color, grid_lw);

  // KITTI
  const char *data_dir = "/data/kitti_raw/2011_09_26";
  const char *seq_name = "2011_09_26_drive_0001_sync";
  kitti_raw_t *kitti = kitti_raw_load(data_dir, seq_name);
  gl_points3d_t *gl_points = gl_points3d_malloc(NULL, 0, 0);

  int pcd_index = 0;
  double time_prev = glfwGetTime();

  while (gui_poll(gui)) {
    double time_dt = glfwGetTime() - time_prev;
    if (pcd_index >= kitti->velodyne->num_timestamps) {
      break;
    }

    if (*gui->key_n || time_dt > 0.1) {
      printf("[pcd_index]: %d\n", pcd_index);
      const timestamp_t ts_start = kitti->velodyne->timestamps_start[pcd_index];
      const timestamp_t ts_end = kitti->velodyne->timestamps_end[pcd_index];
      const char *pcd_path = kitti->velodyne->pcd_paths[pcd_index];

      size_t num_points = 0;
      float *points = kitti_load_points(pcd_path, &num_points);
      float *time_diffs = malloc(sizeof(float) * num_points);
      for (int i = 0; i < num_points; ++i) {
        time_diffs[i] = 0.0;
      }
      pcd_t *pcd = pcd_malloc(ts_start, ts_end, points, time_diffs, num_points);

      vis_kitti_scan(gl_points, points, num_points);
      draw_points3d(gl_points);
      time_prev = glfwGetTime();

      free(points);
      free(time_diffs);
      pcd_free(pcd);

      pcd_index++;
    }

    draw_grid3d(gl_grid);
    draw_points3d(gl_points);
    gui_update(gui);
  }

  // Clean up
  kitti_raw_free(kitti);
  gl_points3d_free(gl_points);
  gl_grid3d_free(gl_grid);
  gui_free(gui);

  return 0;
}

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
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);

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

  solver_t solver;
  solver_setup(&solver);

  // Clean up
  sim_camera_data_free(cam_data);
  // free(cam_views);
  // features_free(features);

  return 0;
}

void test_suite(void) {
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
  MU_ADD_TEST(test_imu_buffer_setup);
  MU_ADD_TEST(test_imu_buffer_add);
  MU_ADD_TEST(test_imu_buffer_clear);
  MU_ADD_TEST(test_imu_buffer_copy);
  MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_imu_initial_attitude);
  MU_ADD_TEST(test_imu_factor_form_F_matrix);
  MU_ADD_TEST(test_imu_factor);
  MU_ADD_TEST(test_joint_factor);
  MU_ADD_TEST(test_marg);
  MU_ADD_TEST(test_visual_odometry_batch);
  MU_ADD_TEST(test_inertial_odometry_batch);
  // MU_ADD_TEST(test_visual_inertial_odometry_batch);
  MU_ADD_TEST(test_icp);
  // MU_ADD_TEST(test_tsf);
  // MU_ADD_TEST(test_assoc_pose_data);
#ifdef USE_CERES
  MU_ADD_TEST(test_ceres_example);
#endif // USE_CERES
  MU_ADD_TEST(test_solver_setup);
  // MU_ADD_TEST(test_solver_eval);
}
MU_RUN_TESTS(test_suite)
