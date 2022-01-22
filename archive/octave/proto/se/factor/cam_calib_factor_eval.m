function [r, jacs] = cam_calib_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 3);

  % Setup return values
  r = zeros(2, 1);
  jacs{1} = zeros(2, 6);  % w.r.t Relative pose T_C0F
  jacs{2} = zeros(2, 6);  % w.r.t Extrinsics T_C0Ci
  jacs{3} = zeros(2, 8);  % w.r.t Camera parameters

  % Map params
  rel_pose = params{1};
  exts = params{2};
  camera = params{3};

  % Project point in world frame to image plane
  T_C0F = tf(rel_pose.param);
  T_C0Ci = tf(exts.param);
  r_FFi = factor.r_FFi;
  r_CiFi = tf_point(inv(T_C0Ci) * T_C0F, r_FFi);
  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);
  z_hat = camera.project(proj_params, dist_params, r_CiFi);

  % Calculate residual
  sqrt_info = factor.sqrt_info;
  z = factor.z;
  r = sqrt_info * (z - z_hat);

  % dr/dzhat * dzhat/dr_CiFi * dr_CiFi/dr_C0F * dr_C0F/dT_C0F
  % -1       *      Jh       *     C_CiC0     * 

  % r_CiFi = inv(T_C0Ci) * r_C0Fi
  % r_CiFi = T_CiC0 * r_C0Fi
  % r_CiFi = C_CiC0 * r_C0Fi + r_CiC0

  % dr/dzhat * dzhat/dr_CiFi * dr_CiFi/dT_CiC0 * dT_CiC0/dT_C0Ci
  % -1       *      Jh       *      eye(3)     *     -C_CiC0

  % Calculate Jacobians
  C_C0F = tf_rot(T_C0F);
  C_C0Ci = tf_rot(T_C0Ci);
  C_CiC0 = C_C0Ci';
  r_C0Fi = tf_point(T_C0F, r_FFi);
  % -- Measurement model jacobian
  Jh = camera.J_proj(proj_params, dist_params, r_CiFi);
  % -- Jacobian w.r.t. relative pose T_C0F
  jacs{1}(1:2, 1:3) = -1 * sqrt_info * Jh * C_CiC0;
  jacs{1}(1:2, 4:6) = -1 * sqrt_info * Jh * C_CiC0 * skew(C_C0F * r_FFi) * -C_C0F;
  % -- Jacobian w.r.t. relative pose T_C0Ci
  jacs{2}(1:2, 1:3) = -1 * sqrt_info * Jh * -C_CiC0;
  jacs{2}(1:2, 4:6) = -1 * sqrt_info * Jh * -C_CiC0 * skew(C_CiC0 * r_C0Fi) *C_CiC0;
  % -- Jacobian w.r.t. camera parameters
  J_cam_params = camera.J_param(proj_params, dist_params, r_CiFi);
  jacs{3} = -1 * sqrt_info * J_cam_params;
endfunction
