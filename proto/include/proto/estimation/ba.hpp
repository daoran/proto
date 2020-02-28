#ifndef PROTO_BA_HPP
#define PROTO_BA_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "proto/core/core.hpp"

namespace proto {

struct pose_t {
  timestamp_t ts = 0;
  quat_t q;
  vec3_t r;

  pose_t() {}

  pose_t(const quat_t &q_, const vec3_t &r_)
    : q{q_}, r{r_} {}

  pose_t(const timestamp_t &ts_, const quat_t &q_, const vec3_t &r_)
    : ts{ts_}, q{q_}, r{r_} {}
};

typedef std::vector<pose_t> poses_t;

void pose_set_quat(pose_t &pose, const quat_t &q) {
  pose.q = q;
}

void pose_set_trans(pose_t &pose, const vec3_t &r) {
  pose.r = r;
}

void pose_print(const std::string &prefix, const pose_t &pose) {
  printf("[%s] ", prefix.c_str());
  printf("q: (%f, %f, %f, %f)", pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z());
  printf("\t");
  printf("r: (%f, %f, %f)\n", pose.r(0), pose.r(1), pose.r(2));
}

mat4_t pose2tf(const pose_t &pose) {
  return tf(pose.q, pose.r);
}

static poses_t load_poses(const std::string &csv_path) {
  FILE *csv_file = fopen(csv_path.c_str(), "r");
  char line[1024] = {0};
  poses_t poses;

  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[1024] = {0};
    double data[7] = {0};
    int index = 0;
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[index] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        index++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    quat_t q{data[0], data[1], data[2], data[3]};
    vec3_t r{data[4], data[5], data[6]};
    poses.emplace_back(q, r);
  }
  fclose(csv_file);

  return poses;
}

static mat3_t load_camera(const std::string &data_path) {
  // Setup csv path
  char cam_csv[1000] = {0};
  strcat(cam_csv, data_path.c_str());
  strcat(cam_csv, "/camera.csv");

  // Parse csv file
  int nb_rows = 0;
  int nb_cols = 0;
  double **cam_K = csv_data(cam_csv, &nb_rows, &nb_cols);
  if (cam_K == NULL) {
    FATAL("Failed to load csv file [%s]!", cam_csv);
  }
  if (nb_rows != 3 || nb_cols != 3) {
    LOG_ERROR("Error while parsing camera file [%s]!", cam_csv);
    LOG_ERROR("-- Expected 3 rows got %d instead!", nb_rows);
    LOG_ERROR("-- Expected 3 cols got %d instead!", nb_cols);
    FATAL("Invalid camera file [%s]!", cam_csv);
  }

  // Flatten 2D array to 1D array
  mat3_t K;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      K(i, j) = cam_K[i][j];
    }
    free(cam_K[i]);
  }
  free(cam_K);

  return K;
}

static poses_t load_camera_poses(const std::string &data_path) {
  char cam_poses_csv[1000] = {0};
  strcat(cam_poses_csv, data_path.c_str());
  strcat(cam_poses_csv, "/camera_poses.csv");
  return load_poses(cam_poses_csv);
}

static poses_t load_target_pose(const std::string &data_path) {
  char target_pose_csv[1000] = {0};
  strcat(target_pose_csv, data_path.c_str());
  strcat(target_pose_csv, "/target_pose.csv");
  return load_poses(target_pose_csv);
}

struct keypoints_t {
  double **data;
  int size;
};

void keypoints_delete(keypoints_t *keypoints) {
  for (int i = 0; i < keypoints->size; i++) {
    free(keypoints->data[i]);
  }
  free(keypoints->data);
  free(keypoints);
}

void keypoints_print(const keypoints_t &keypoints) {
  printf("nb_keypoints: %d\n", keypoints.size);
  printf("keypoints:\n");
  for (int i = 0; i < keypoints.size; i++) {
    printf("-- (%f, %f)\n", keypoints.data[i][0], keypoints.data[i][1]);
  }
}

static keypoints_t parse_keypoints_line(char *line) {
  keypoints_t keypoints;
  keypoints.data = NULL;
  keypoints.size = 0;

  char entry[100] = {0};
  int kp_ready = 0;
  double kp[2] = {0};
  int kp_index = 0;

  // Parse line
  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      // Initialize keypoints
      if (keypoints.data == NULL) {
        size_t array_size = strtod(entry, NULL);
        keypoints.data = (double **) calloc(array_size, sizeof(double *));
        keypoints.size = array_size / 2.0;

      } else { // Parse keypoint
        if (kp_ready == 0) {
          kp[0] = strtod(entry, NULL);
          kp_ready = 1;

        } else {
          kp[1] = strtod(entry, NULL);
          keypoints.data[kp_index] = (double *) malloc(sizeof(double) * 2);
          keypoints.data[kp_index][0] = kp[0];
          keypoints.data[kp_index][1] = kp[1];

          kp_ready = 0;
          kp_index++;
        }
      }

      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return keypoints;
}

static std::vector<keypoints_t> load_keypoints(const std::string &data_path) {
  char keypoints_csv[1000] = {0};
  strcat(keypoints_csv, data_path.c_str());
  strcat(keypoints_csv, "/keypoints.csv");

  FILE *csv_file = fopen(keypoints_csv, "r");
  std::vector<keypoints_t> keypoints;

  char line[1024] = {0};
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    keypoints.push_back(parse_keypoints_line(line));
  }
  fclose(csv_file);

  return keypoints;
}

static double **load_points(const std::string &data_path, int *nb_points) {
  char points_csv[1000] = {0};
  strcat(points_csv, data_path.c_str());
  strcat(points_csv, "/points.csv");

  // Initialize memory for points
  *nb_points = csv_rows(points_csv);
  double **points = (double **) malloc(sizeof(double *) * *nb_points);
  for (int i = 0; i < *nb_points; i++) {
    points[i] = (double *) malloc(sizeof(double) * 3);
  }

  // Load file
  FILE *infile = fopen(points_csv, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[1024] = {0};
  size_t len_max = 1024;
  int point_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        points[point_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    point_idx++;
  }

  // Cleanup
  fclose(infile);

  return points;
}

static int **load_point_ids(const std::string &data_path, int *nb_points) {
  char csv_path[1000] = {0};
  strcat(csv_path, data_path.c_str());
  strcat(csv_path, "/point_ids.csv");
  return load_iarrays(csv_path, nb_points);
}

struct ba_data_t {
  mat3_t cam_K;

  poses_t cam_poses;
  pose_t target_pose;
  int nb_frames;

  std::vector<keypoints_t> keypoints;

  int **point_ids;
  int nb_ids;

  double **points;
  int nb_points;

  ba_data_t(const std::string &data_path) {
    cam_K = load_camera(data_path);
    cam_poses = load_camera_poses(data_path);
    target_pose = load_target_pose(data_path)[0];
    nb_frames = cam_poses.size();
    keypoints = load_keypoints(data_path);
    point_ids = load_point_ids(data_path, &nb_ids);
    points = load_points(data_path, &nb_points);
  }

  ~ba_data_t(){
    // Point IDs
    for (int i = 0; i < nb_frames; i++) {
      free(point_ids[i]);
    }
    free(point_ids);

    // Points
    for (int i = 0; i < nb_points; i++) {
      free(points[i]);
    }
    free(points);
  }

} typedef ba_data_t;

int ba_residual_size(ba_data_t &data) {
  // Calculate residual size
  int r_size = 0;
  for (int k = 0; k < data.nb_frames; k++) {
    r_size += data.point_ids[k][0];
  }
  r_size = r_size * 2;
  // ^ Scale 2 because each pixel error are size 2

  return r_size;
}

double *ba_residuals(ba_data_t &data, int *r_size) {
  // Initialize memory for residuals
  *r_size = ba_residual_size(data);
  double *r = (double *) calloc(*r_size, sizeof(double));

  // // Target pose
  // mat4_t T_WT = pose2tf(data.target_pose);

  // Loop over time
  int res_idx = 0; // Residual index
  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose
    mat4_t T_WC = pose2tf(data.cam_poses[k]);

    // Invert camera pose T_WC to T_CW
    mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};

      // Transform point in world frame to camera frame
      const vec3_t p_C = (T_CW * p_W.homogeneous()).head(3);

      // Project point in camera frame down to image plane
      vec2_t z_hat{0.0, 0.0};
      // pinhole_project(data.cam_K, p_C, z_hat);

      // Calculate reprojection error
      const vec2_t z{data.keypoints[k].data[i]};
      r[res_idx] = z(0) - z_hat(0);
      r[res_idx + 1] = z(1) - z_hat(1);
      res_idx += 2;
    }
  }

  return r;
}

static mat2_t J_intrinsics_point(const mat3_t K) {
  // J = [K[0, 0], 0.0,
  // 		  0.0, K[1, 1]];
  mat2_t J = zeros(2, 2);
  J(0, 0) = K(0);
  J(1, 1) = K(2);
  return J;
}

static matx_t J_project(const vec3_t p_C) {
  const double x = p_C(0);
  const double y = p_C(1);
  const double z = p_C(2);

  // J = [1 / z, 0, -x / z^2,
  // 		  0, 1 / z, -y / z^2];
  matx_t J = zeros(2, 3);
  J(0, 0) = 1.0 / z;
  J(0, 2) = 1.0 / z;
  J(1, 1) = -x / z * z;
  J(1, 2) = -y / z * z;
  return J;
}

static mat3_t J_camera_rotation(const quat_t q_WC,
                                const vec3_t r_WC,
                                const vec3_t p_W) {
  // Convert quaternion to rotatoin matrix
  mat3_t C_WC = q_WC.toRotationMatrix();
  mat3_t J = C_WC * skew(p_W - r_WC);
  return J;
}

static mat3_t J_camera_translation(const quat_t q_WC) {
  // Convert quaternion to rotatoin matrix
  mat3_t C_WC = q_WC.toRotationMatrix();

  // J = -C_CW
  return -C_WC.transpose();
}

static mat3_t J_target_point(const quat_t q_WC) {
  // Convert quaternion to rotatoin matrix
  mat3_t C_WC = q_WC.toRotationMatrix();

  // J = C_CW
  return C_WC.transpose();
}

matx_t ba_jacobian(ba_data_t &data, int *J_rows, int *J_cols) {
  // Initialize memory for jacobian
  *J_rows = ba_residual_size(data);
  *J_cols = (data.nb_frames * 6) + (data.nb_points * 3);
  matx_t J = zeros(*J_rows, *J_cols);

  // Loop over camera poses
  int pose_idx = 0;
  int meas_idx = 0;

  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose
    mat4_t T_WC = pose2tf(data.cam_poses[k]);
    quat_t q_WC = tf_quat(T_WC);
    vec3_t r_WC = tf_trans(T_WC);

    // Invert T_WC to T_CW
    mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    // Loop over observations at time k
    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};

      // Transform point in world frame to camera frame
      const vec3_t p_C = (T_CW * p_W.homogeneous()).head(3);

      // Camera pose jacobian
      // -- Setup row start, row end, column start and column end
      const int rs = meas_idx * 2;
      int cs = pose_idx * 6;
      int ce = cs + 5;

      // -- Form jacobians
      const mat2_t J_K = J_intrinsics_point(data.cam_K);
      const matx_t J_P = J_project(p_C);
      const mat3_t J_C = J_camera_rotation(q_WC, r_WC, p_W);
      const mat3_t J_r = J_camera_translation(q_WC);

      // J_cam_rot = -1 * J_K * J_P * J_C;
      const matx_t J_cam_rot = -1 * J_K * J_P * J_C;

      // J_cam_pos = -1 * J_K * J_P * J_r;
      const matx_t J_cam_pos = -1 * J_K * J_P * J_r;

      // -- Fill in the big jacobian
      J.block(rs, cs, 2, 3) = J_cam_rot;
      J.block(rs, cs + 3, 2, 3) = J_cam_rot;

      // Point jacobian
      // -- Setup row start, row end, column start and column end
      cs = (data.nb_frames * 6) + point_ids[i] * 3;
      ce = cs + 2;

      // -- Form jacobians
      matx_t J_point = -1 * J_K * J_P * J_target_point(q_WC);

      // -- Fill in the big jacobian
      J.block(rs, cs, 2, 3) = J_point;

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

// void ba_update(
//     ba_data_t *data, double *e, int e_size, double *E, int E_rows, int E_cols) {
//   assert(e_size == E_rows);
//   // Form weight matrix
//   // W = diag(repmat(sigma, data->nb_measurements, 1));
//
//   // Solve Gauss-Newton system [H dx = g]: Solve for dx
//   // H = (E' * W * E);
//   double *E_t = mat_new(E_cols, E_rows);
//   double *H = mat_new(E_cols, E_cols);
//   mat_transpose(E, E_rows, E_cols, E_t);
//   dot(E_t, E_cols, E_rows, E, E_rows, E_cols, H);
//
//   // g = -E' * W * e;
//   double *g = vec_new(E_cols);
//   mat_scale(E_t, E_cols, E_rows, -1.0);
//   dot(E_t, E_cols, E_rows, e, e_size, 1, g);
//   free(E_t);
//
//   // #<{(| dx = pinv(H) * g; |)}>#
//   // double *H_inv = mat_new(E_cols, E_cols);
//   // double *dx = vec_new(E_cols);
//   // pinv(H, E_cols, E_cols, H_inv);
//   // dot(H_inv, E_cols, E_cols, g, E_cols, 1, dx);
//   // free(H);
//   // free(H_inv);
//   // free(g);
//
//   // #<{(| Update camera poses |)}>#
//   // for (int k = 0; k < data->nb_frames; k++) {
//   //   const int s = k * 6;
//   //
//   //   #<{(| Update camera rotation |)}>#
//   //   #<{(| dq = quatdelta(dalpha) |)}>#
//   //   #<{(| q_WC_k = quatmul(dq, q_WC_k) |)}>#
//   //   const double dalpha[3] = {dx[s], dx[s + 1], dx[s + 2]};
//   //   double dq[4] = {0};
//   //   double q_new[4] = {0};
//   //   quatdelta(dalpha, dq);
//   //   quatmul(dq, data->cam_poses[k].q, q_new);
//   //   data->cam_poses[k].q[0] = q_new[0];
//   //   data->cam_poses[k].q[1] = q_new[1];
//   //   data->cam_poses[k].q[2] = q_new[2];
//   //   data->cam_poses[k].q[3] = q_new[3];
//   //
//   //   #<{(| Update camera position |)}>#
//   //   #<{(| r_WC_k += dr_WC |)}>#
//   //   const double dr_WC[3] = {dx[s + 3], dx[s + 4], dx[s + 5]};
//   //   data->cam_poses[k].r[0] += dr_WC[0];
//   //   data->cam_poses[k].r[1] += dr_WC[1];
//   //   data->cam_poses[k].r[2] += dr_WC[2];
//   // }
//   //
//   // #<{(| Update points |)}>#
//   // for (int i = 0; i < data->nb_points; i++) {
//   //   const int s = (data->nb_frames * 6) + (i * 3);
//   //   const double dp_W[3] = {dx[s], dx[s + 1], dx[s + 2]};
//   //   data->points[i][0] += dp_W[0];
//   //   data->points[i][1] += dp_W[1];
//   //   data->points[i][2] += dp_W[2];
//   // }
//
//   // Clean up
//   free(dx);
// }
//
// double ba_cost(const double *e, const int length) {
//   // cost = 0.5 * e' * e
//   double cost = 0.0;
//   dot(e, 1, length, e, length, 1, &cost);
//   return cost * 0.5;
// }
//
// void ba_solve(ba_data_t *data) {
//   int max_iter = 2;
//   double cost_prev = 0.0;
//
//   for (int iter = 0; iter < max_iter; iter++) {
//     // Residuals
//     int e_size = 0;
//     double *e = ba_residuals(data, &e_size);
//
//     // Jacobians
//     int E_rows = 0;
//     int E_cols = 0;
//     double *E = ba_jacobian(data, &E_rows, &E_cols);
//
//     // Update and calculate cost
//     ba_update(data, e, e_size, E, E_rows, E_cols);
//     const double cost = ba_cost(e, e_size);
//     printf("iter: %d\t cost: %.4e\n", iter, cost);
//     free(e);
//     free(E);
//
//     // Termination criteria
//     double cost_diff = fabs(cost - cost_prev);
//     if (cost_diff < 1.0e-6) {
//       printf("Done!\n");
//       break;
//     }
//     cost_prev = cost;
//   }
// }

} // namespace proto
#endif // PROTO_BA_HPP
