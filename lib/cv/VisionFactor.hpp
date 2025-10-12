

struct VisionFactor : ceres::CostFunction {
  timestamp_t ts;
  int cam_id = -1;
  size_t feature_id = 0;
  real_t z[2] = {0};
  real_t sqrt_info[2 * 2] = {0};

  /** Constructor **/
  VisionFactor() = delete;

  /** Constructor **/
  VisionFactor(const timestamp_t ts_,
               const int cam_id_,
               const size_t feature_id_,
               const real_t z_[2])
      : ts{ts_}, cam_id{cam_id_}, feature_id{feature_id_} {
    z[0] = z_[0];
    z[1] = z_[1];
    sqrt_info[0] = 1;
    sqrt_info[1] = 0;
    sqrt_info[2] = 0;
    sqrt_info[3] = 1;

    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7); // Body pose
    block_sizes->push_back(7); // Camera extrinsic
    block_sizes->push_back(3); // Feature position
    block_sizes->push_back(8); // Camera intrinsic
  }

  /** Destructor **/
  virtual ~VisionFactor() = default;

  /** Evaluate **/
  bool Evaluate(double const *const *params, double *res, double **jacs) const {
    // Map parameters
    TF(params[0], T_WB);
    TF(params[1], T_BCi);
    const double *p_W = params[2];
    const double *cam = params[3];

    // Transform feature from world to camera frame
    TF_INV(T_BCi, T_CiB);
    TF_INV(T_WB, T_BW);
    TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
    TF_POINT(T_CiW, p_W, p_Ci);

    // Project to image plane
    bool valid = true;
    real_t z_hat[2] = {0};
    const project_func_t proj_func = pinhole_radtan4_project;
    proj_func(cam, p_Ci, z_hat);

    bool x_ok = z_hat[0] > 0 && z_hat[0] < 752;
    bool y_ok = z_hat[1] > 0 && z_hat[1] < 480;
    if (p_Ci[2] < 0.01) {
      valid = false;
    } else if (!x_ok || !y_ok) {
      valid = false;
    }

    // Calculate residuals
    // -- Residual
    double r[2] = {0};
    r[0] = z[0] - z_hat[0];
    r[1] = z[1] - z_hat[1];
    // -- Weighted residual
    dot(sqrt_info, 2, 2, r, 2, 1, res);

    // Calculate jacobians
    // -- Form: -1 * sqrt_info
    real_t neg_sqrt_info[2 * 2] = {0};
    mat_copy(sqrt_info, 2, 2, neg_sqrt_info);
    neg_sqrt_info[0] *= -1.0;
    neg_sqrt_info[3] *= -1.0;
    // -- Form: Jh_ = -1 * sqrt_info * Jh
    real_t Jh[2 * 3] = {0};
    real_t Jh_[2 * 3] = {0};
    pinhole_radtan4_project_jacobian(cam, p_Ci, Jh);
    dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_);
    // -- Form: J_cam_params
    real_t J_cam_params[2 * 8] = {0};
    pinhole_radtan4_params_jacobian(cam, p_Ci, J_cam_params);
    // -- Form minimal Jacobians
    double mJ0[2 * 6] = {0};
    double mJ1[2 * 6] = {0};
    double mJ2[2 * 3] = {0};
    double mJ3[2 * 8] = {0};
    if (valid) {
      camera_factor_pose_jacobian(Jh_, T_WB, T_BCi, p_W, mJ0);
      camera_factor_extrinsic_jacobian(Jh_, T_BCi, p_Ci, mJ1);
      camera_factor_feature_jacobian(Jh_, T_WB, T_BCi, mJ2);
      camera_factor_camera_jacobian(neg_sqrt_info, J_cam_params, mJ3);
    }
    // -- Form global Jacobians
    if (jacs) {
      if (jacs[0]) {
        memset(jacs[0], 0, sizeof(double) * 2 * 7);
        mat_block_set(jacs[0], 7, 0, 1, 0, 5, mJ0);
      }

      if (jacs[1]) {
        memset(jacs[1], 0, sizeof(double) * 2 * 7);
        mat_block_set(jacs[1], 7, 0, 1, 0, 5, mJ1);
      }

      if (jacs[2]) {
        memset(jacs[2], 0, sizeof(double) * 2 * 3);
        mat_copy(mJ2, 2, 3, jacs[2]);
      }

      if (jacs[3]) {
        memset(jacs[3], 0, sizeof(double) * 2 * 8);
        mat_copy(mJ3, 2, 3, jacs[3]);
      }
    }

    return true;
  }

  /**
   * Pose jacobian
   */
  static void camera_factor_pose_jacobian(const real_t Jh_w[2 * 3],
                                          const real_t T_WB[3 * 3],
                                          const real_t T_BC[3 * 3],
                                          const real_t p_W[3],
                                          real_t J[2 * 6]) {
    assert(Jh_w != NULL);
    assert(T_BC != NULL);
    assert(T_WB != NULL);
    assert(p_W != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J_pos = Jh_w * C_CB * -C_BW;
    // J_rot = Jh_w * C_CB * C_BW * hat(p_W - r_WB) * -C_WB;
    // J = [J_pos, J_rot];

    // Setup
    real_t C_BW[3 * 3] = {0};
    real_t C_CB[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};

    TF_ROT(T_WB, C_WB);
    TF_ROT(T_BC, C_BC);
    mat_transpose(C_WB, 3, 3, C_BW);
    mat_transpose(C_BC, 3, 3, C_CB);
    dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

    // Form: -C_BW
    real_t neg_C_BW[3 * 3] = {0};
    mat_copy(C_BW, 3, 3, neg_C_BW);
    mat_scale(neg_C_BW, 3, 3, -1.0);

    // Form: -C_CW
    real_t neg_C_CW[3 * 3] = {0};
    dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

    // Form: -C_WB
    real_t neg_C_WB[3 * 3] = {0};
    mat_copy(C_WB, 3, 3, neg_C_WB);
    mat_scale(neg_C_WB, 3, 3, -1.0);

    // Form: C_CB * -C_BW * hat(p_W - r_WB) * -C_WB
    real_t p[3] = {0};
    real_t S[3 * 3] = {0};
    TF_TRANS(T_WB, r_WB);
    vec_sub(p_W, r_WB, p, 3);
    hat(p, S);

    real_t A[3 * 3] = {0};
    real_t B[3 * 3] = {0};
    dot(neg_C_CW, 3, 3, S, 3, 3, A);
    dot(A, 3, 3, neg_C_WB, 3, 3, B);

    // Form: J_pos = Jh_w * C_CB * -C_BW;
    real_t J_pos[2 * 3] = {0};
    dot(Jh_w, 2, 3, neg_C_CW, 3, 3, J_pos);

    J[0] = J_pos[0];
    J[1] = J_pos[1];
    J[2] = J_pos[2];

    J[6] = J_pos[3];
    J[7] = J_pos[4];
    J[8] = J_pos[5];

    // Form: J_rot = Jh_w * C_CB * -C_BW * hat(p_W - r_WB) * -C_WB;
    real_t J_rot[2 * 3] = {0};
    dot(Jh_w, 2, 3, B, 3, 3, J_rot);

    J[3] = J_rot[0];
    J[4] = J_rot[1];
    J[5] = J_rot[2];

    J[9] = J_rot[3];
    J[10] = J_rot[4];
    J[11] = J_rot[5];
  }

  /**
   * Body-camera extrinsic jacobian
   */
  static void camera_factor_extrinsic_jacobian(const real_t Jh_w[2 * 3],
                                               const real_t T_BC[4 * 4],
                                               const real_t p_C[3],
                                               real_t J[2 * 6]) {
    assert(Jh_w != NULL);
    assert(T_BC != NULL);
    assert(p_C != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J_pos = Jh_w * -C_CB;
    // J_rot = Jh_w * C_CB * hat(C_BC * p_C);

    // Setup
    real_t C_BC[3 * 3] = {0};
    real_t C_CB[3 * 3] = {0};
    real_t C_BW[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};

    tf_rot_get(T_BC, C_BC);
    mat_transpose(C_BC, 3, 3, C_CB);
    dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

    // Form: -C_CB
    real_t neg_C_CB[3 * 3] = {0};
    mat_copy(C_CB, 3, 3, neg_C_CB);
    mat_scale(neg_C_CB, 3, 3, -1.0);

    // Form: -C_BC
    real_t neg_C_BC[3 * 3] = {0};
    mat_copy(C_BC, 3, 3, neg_C_BC);
    mat_scale(neg_C_BC, 3, 3, -1.0);

    // Form: -C_CB * hat(C_BC * p_C) * -C_BC
    real_t p[3] = {0};
    real_t S[3 * 3] = {0};
    dot(C_BC, 3, 3, p_C, 3, 1, p);
    hat(p, S);

    real_t A[3 * 3] = {0};
    real_t B[3 * 3] = {0};
    dot(neg_C_CB, 3, 3, S, 3, 3, A);
    dot(A, 3, 3, neg_C_BC, 3, 3, B);

    // Form: J_rot = Jh_w * -C_CB;
    real_t J_pos[2 * 3] = {0};
    dot(Jh_w, 2, 3, neg_C_CB, 3, 3, J_pos);

    J[0] = J_pos[0];
    J[1] = J_pos[1];
    J[2] = J_pos[2];

    J[6] = J_pos[3];
    J[7] = J_pos[4];
    J[8] = J_pos[5];

    // Form: J_rot = Jh_w * -C_CB * hat(C_BC * p_C) * -C_BC;
    real_t J_rot[2 * 3] = {0};
    dot(Jh_w, 2, 3, B, 3, 3, J_rot);

    J[3] = J_rot[0];
    J[4] = J_rot[1];
    J[5] = J_rot[2];

    J[9] = J_rot[3];
    J[10] = J_rot[4];
    J[11] = J_rot[5];
  }

  /**
   * Camera parameters jacobian
   */
  static void camera_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                            const real_t J_cam_params[2 * 8],
                                            real_t J[2 * 8]) {
    assert(neg_sqrt_info != NULL);
    assert(J_cam_params != NULL);
    assert(J != NULL);

    // J = -1 * sqrt_info * J_cam_params;
    dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
  }

  /**
   * Feature jacobian
   */
  static void camera_factor_feature_jacobian(const real_t Jh_w[2 * 3],
                                             const real_t T_WB[4 * 4],
                                             const real_t T_BC[4 * 4],
                                             real_t J[2 * 3]) {
    if (J == NULL) {
      return;
    }
    assert(Jh_w != NULL);
    assert(T_WB != NULL);
    assert(T_BC != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J = Jh_w * C_CW;

    // Setup
    real_t T_WC[4 * 4] = {0};
    real_t C_WC[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};
    dot(T_WB, 4, 4, T_BC, 4, 4, T_WC);
    tf_rot_get(T_WC, C_WC);
    mat_transpose(C_WC, 3, 3, C_CW);

    // Form: J = -1 * sqrt_info * Jh * C_CW;
    dot(Jh_w, 2, 3, C_CW, 3, 3, J);
  }
};
