
/**
 * Filter features by triangulating them via a stereo-pair and see if the
 * reprojection error is reasonable.
 */
void reproj_filter(const project_func_t cam_i_proj_func,
                   const int cam_i_res[2],
                   const real_t *cam_i_params,
                   const real_t *cam_j_params,
                   const real_t T_C0Ci[4 * 4],
                   const real_t T_C0Cj[4 * 4],
                   const std::vector<cv::KeyPoint> &kps_i,
                   const std::vector<cv::KeyPoint> &kps_j,
                   std::vector<cv::Point3d> &points,
                   std::vector<bool> &inliers,
                   const real_t reproj_threshold = 0.5) {
  // Form camera i and camera j extrinsics T_CiCj
  TF_INV(T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiCj, 2, T_CiC0, T_C0Cj);

  // Form Projection matrices P_i and P_j
  TF_IDENTITY(T_eye);
  real_t P_i[3 * 4] = {0};
  real_t P_j[3 * 4] = {0};
  pinhole_projection_matrix(cam_i_params, T_eye, P_i);
  pinhole_projection_matrix(cam_j_params, T_CiCj, P_j);

  // Check features
  for (size_t n = 0; n < kps_i.size(); n++) {
    // Triangulate feature
    const real_t z_i[2] = {kps_i[n].pt.x, kps_i[n].pt.y};
    const real_t z_j[2] = {kps_j[n].pt.x, kps_j[n].pt.y};

    real_t uz_i[2] = {0};
    real_t uz_j[2] = {0};
    pinhole_radtan4_undistort(cam_i_params, z_i, uz_i);
    pinhole_radtan4_undistort(cam_j_params, z_j, uz_j);

    real_t p_Ci[3] = {0};
    linear_triangulation(P_i, P_j, uz_i, uz_j, p_Ci);
    points.emplace_back(p_Ci[0], p_Ci[1], p_Ci[2]);

    // Check feature depth
    if (p_Ci[2] < 0.0) {
      inliers.push_back(false);
      continue;
    }

    // Reproject feature into camera i
    real_t z_i_hat[2] = {0};
    cam_i_proj_func(cam_i_params, p_Ci, z_i_hat);

    // Check image point bounds
    const bool x_ok = (z_i_hat[0] < cam_i_res[0] && z_i_hat[0] > 0);
    const bool y_ok = (z_i_hat[1] < cam_i_res[1] && z_i_hat[1] > 0);
    if (!x_ok || !y_ok) {
      inliers.push_back(false);
      continue;
    }

    // Check reprojection error
    const real_t r[2] = {z_i[0] - z_i_hat[0], z_i[1] - z_i_hat[1]};
    const real_t reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
    if (reproj_error > reproj_threshold) {
      inliers.push_back(false);
      continue;
    }

    // Passed all tests
    inliers.push_back(true);
  }
}
