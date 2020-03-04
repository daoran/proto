#include "proto/estimation/ba.hpp"

namespace proto {

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

ba_data_t::ba_data_t(const std::string &data_path) {
  cam_K = load_camera(data_path);
  cam_poses = load_camera_poses(data_path);
  target_pose = load_target_pose(data_path)[0];
  nb_frames = cam_poses.size();
  keypoints = load_keypoints(data_path);
  point_ids = load_point_ids(data_path, &nb_ids);
  points = load_points(data_path, &nb_points);
}

ba_data_t::~ba_data_t() {
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

static mat2_t J_intrinsics(const mat3_t &K) {
  // J = [K[0, 0], 0.0,
  //       0.0, K[1, 1]];
  mat2_t J = zeros(2, 2);
  J(0, 0) = K(0, 0);
  J(1, 1) = K(1, 1);
  return J;
}

static matx_t J_project(const vec3_t &p_C) {
  const double x = p_C(0);
  const double y = p_C(1);
  const double z = p_C(2);

  // J = [1 / z, 0, -x / z^2,
  //       0, 1 / z, -y / z^2];
  matx_t J = zeros(2, 3);
  J(0, 0) = 1.0 / z;
  J(1, 1) = 1.0 / z;
  J(0, 2) = -x / (z * z);
  J(1, 2) = -y / (z * z);
  return J;
}

static mat3_t J_camera_rotation(const quat_t &q_WC,
                         const vec3_t &r_WC,
                         const vec3_t &p_W) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return C_CW * skew(p_W - r_WC);
}

static mat3_t J_camera_translation(const quat_t &q_WC) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return -C_CW;
}

static mat3_t J_target_point(const quat_t &q_WC) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return C_CW;
}

int check_J_cam_pose(const mat3_t &cam_K,
                     const mat4_t &T_WC,
                     const vec3_t &p_W,
                     const mat_t<2, 6> &J_cam_pose,
                     const double step_size = 1e-3,
                     const double threshold = 1e-2) {
  const vec2_t z{0.0, 0.0};
  const vec4_t hp_W = p_W.homogeneous();

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const mat4_t T_WC_fd = tf_perturb_rot(T_WC, step_size, i);
    const mat4_t T_CW_fd = T_WC_fd.inverse();
    const vec3_t p_C_fd = (T_CW_fd * hp_W).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const mat4_t T_WC_bd = tf_perturb_rot(T_WC, -step_size, i);
    const mat4_t T_CW_bd = T_WC_bd.inverse();
    const vec3_t p_C_bd = (T_CW_bd * hp_W).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    // Calculate central finite difference
    fdiff.block(0, i, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const mat4_t T_WC_fd = tf_perturb_trans(T_WC, step_size, i);
    const mat4_t T_CW_fd = T_WC_fd.inverse();
    const vec3_t p_C_fd = (T_CW_fd * hp_W).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const mat4_t T_WC_bd = tf_perturb_trans(T_WC, -step_size, i);
    const mat4_t T_CW_bd = T_WC_bd.inverse();
    const vec3_t p_C_bd = (T_CW_bd * hp_W).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    // Calculate central finite difference
    fdiff.block(0, i + 3, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_cam_pose", fdiff, J_cam_pose, threshold, true);
}

int check_J_point(const mat3_t &cam_K,
                  const mat4_t &T_WC,
                  const vec3_t &p_W,
                  const mat_t<2, 3> &J_point,
                  const double step_size = 1e-10,
                  const double threshold = 1e-2) {
  const vec2_t z{0.0, 0.0};
  const mat4_t T_CW = T_WC.inverse();
  matx_t fdiff = zeros(2, 3);
  mat3_t dr = I(3) * step_size;

  // Perturb landmark
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const vec3_t p_W_fd = p_W + dr.col(i);
    const vec4_t hp_W_fd = p_W_fd.homogeneous();
    const vec3_t p_C_fd = (T_CW * hp_W_fd).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const vec3_t p_W_bd = p_W - dr.col(i);
    const vec4_t hp_W_bd = p_W_bd.homogeneous();
    const vec3_t p_C_bd = (T_CW * hp_W_bd).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    // Calculate central finite difference
    fdiff.block(0, i, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_point", fdiff, J_point, threshold, true);
}

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

vecx_t ba_residuals(ba_data_t &data) {
  // Initialize memory for residuals
  int r_size = ba_residual_size(data);
  vecx_t r{r_size};

  // Loop over time
  int res_idx = 0; // Residual index
  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose and its inverse
    const mat4_t T_WC = data.cam_poses[k].tf();
    const mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame and transform to camera frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};
      const vec3_t p_C = tf_point(T_CW, p_W);

      // Project point in camera frame down to image plane
      const vec3_t x{p_C(0) / p_C(2), p_C(1) / p_C(2), 1.0};
      const vec2_t z_hat = (data.cam_K * x).head(2);

      // Calculate reprojection error
      const vec2_t z = data.keypoints[k][i];
      r.block(res_idx, 0, 2, 1) = z - z_hat;
      res_idx += 2;
    }
  }

  return r;
}

matx_t ba_jacobian(ba_data_t &data) {
  // Initialize memory for jacobian
  int J_rows = ba_residual_size(data);
  int J_cols = (data.nb_frames * 6) + (data.nb_points * 3);
  matx_t J = zeros(J_rows, J_cols);

  // Loop over camera poses
  int pose_idx = 0;
  int meas_idx = 0;

  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose
    const mat4_t T_WC = data.cam_poses[k].tf();
    const quat_t q_WC = tf_quat(T_WC);
    const vec3_t r_WC = tf_trans(T_WC);
    const mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    // Loop over observations at time k
    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame and transform to camera frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};
      const vec3_t p_C = tf_point(T_CW, p_W);

      // Camera pose jacobian
      const int rs = meas_idx * 2;
      int cs = pose_idx * 6;
      const mat2_t J_K = J_intrinsics(data.cam_K);
      const matx_t J_P = J_project(p_C);
      const mat3_t J_C = J_camera_rotation(q_WC, r_WC, p_W);
      const mat3_t J_r = J_camera_translation(q_WC);
      const matx_t J_cam_rot = -1 * J_K * J_P * J_C;
      const matx_t J_cam_pos = -1 * J_K * J_P * J_r;
      J.block(rs, cs, 2, 3) = J_cam_rot;
      J.block(rs, cs + 3, 2, 3) = J_cam_pos;

      // Point jacobian
      cs = (data.nb_frames * 6) + point_ids[i] * 3;
      const matx_t J_point = -1 * J_K * J_P * J_target_point(q_WC);
      J.block(rs, cs, 2, 3) = J_point;

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

void ba_update(ba_data_t &data, const vecx_t &e, const matx_t &E) {
  // const double lambda = 10.0;  // LM damping term
  const double lambda = 0.001;  // LM damping term

  // Form weight matrix
  // W = diag(repmat(sigma, data->nb_measurements, 1));

  // Solve Gauss-Newton system [H dx = g]: Solve for dx
  matx_t H = E.transpose() * E; // Hessian approx: H = J^t J
  matx_t H_diag = (H.diagonal().asDiagonal());
  // H = H + lambda * I(E.cols());  // original LM damping
  H = H + lambda * H_diag;  // R. Fletcher trust region mod
  const vecx_t g = -E.transpose() * e;

  // const vecx_t dx = H.inverse() * g;  // Slow inverse
  const vecx_t dx = H.ldlt().solve(g);   // Cholesky decomp

  // Update camera poses
  for (int k = 0; k < data.nb_frames; k++) {
    const int s = k * 6;

    // Update camera rotation
    const vec3_t dalpha{dx(s), dx(s + 1), dx(s + 2)};
    const quat_t q = data.cam_poses[k].rot();
    const quat_t dq = quat_delta(dalpha);
    data.cam_poses[k].set_rot(dq * q);

    // Update camera position
    const vec3_t r_WC = data.cam_poses[k].trans();
    const vec3_t dr_WC{dx(s + 3), dx(s + 4), dx(s + 5)};
    data.cam_poses[k].set_trans(r_WC + dr_WC);
  }

  // Update points
  for (int i = 0; i < data.nb_points; i++) {
    const int s = (data.nb_frames * 6) + (i * 3);
    const vec3_t dp_W{dx(s), dx(s + 1), dx(s + 2)};
    data.points[i][0] += dp_W(0);
    data.points[i][1] += dp_W(1);
    data.points[i][2] += dp_W(2);
  }
}

double ba_cost(const vecx_t &e) {
  return 0.5 * e.transpose() * e;
}

void ba_solve(ba_data_t &data) {
  int max_iter = 10;
  double cost_prev = 0.0;

  for (int iter = 0; iter < max_iter; iter++) {
    const vecx_t e = ba_residuals(data);
    const matx_t E = ba_jacobian(data);
    ba_update(data, e, E);

    const double cost = ba_cost(e);
    printf("iter: %d\t cost: %.4e\n", iter, cost);

    // Termination criteria
    double cost_diff = fabs(cost - cost_prev);
    if (cost_diff < 1.0e-6) {
      printf("Done!\n");
      break;
    }
    cost_prev = cost;
  }
}

} // namespace proto
