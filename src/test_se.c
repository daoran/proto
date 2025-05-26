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

void test_suite(void) {
  // MU_ADD_TEST(test_icp);
}
MU_RUN_TESTS(test_suite)
