function [r, jacs] = cam_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 4);

  % Setup return values
  r = zeros(2, 1);
  jacs{1} = zeros(2, 6);  % w.r.t Body pose T_WB
  jacs{2} = zeros(2, 6);  % w.r.t Body-camera pose T_BCi
  jacs{3} = zeros(2, 3);  % w.r.t Feature r_W
  jacs{4} = zeros(2, 8);  % w.r.t Camera parameters

  % Map params
  pose = params{1};
  cam_exts = params{2};
  feature = params{3};
  camera = params{4};
  assert(isfield(camera, "project"));
  assert(isfield(camera, "J_proj"));
  assert(isfield(camera, "J_param"));

  % Project point in world frame to image plane
  T_WB = tf(pose.param);
  T_BCi = tf(cam_exts.param);
  p_W = feature.param;
  p_C = tf_point(inv(T_WB * T_BCi), p_W);
  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);
  z_hat = camera.project(proj_params, dist_params, p_C);

  % Calculate residual
  sqrt_info = factor.sqrt_info;
  z = factor.z;
  r = sqrt_info * (z - z_hat);

  % Calculate Jacobians
  C_BCi = tf_rot(T_BCi);
  C_WB = tf_rot(T_WB);
  C_CB = C_BCi';
  C_BW = C_WB';
  C_CW = C_CB * C_WB';
  r_WB = tf_trans(T_WB);
  % -- Measurement model jacobian
  Jh = camera.J_proj(proj_params, dist_params, p_C);
  % -- Jacobian w.r.t. pose T_WB
  jacs{1}(1:2, 1:3) = -1 * sqrt_info * Jh * C_CB * -C_BW;
  jacs{1}(1:2, 4:6) = -1 * sqrt_info * Jh * C_CB * -C_BW * skew(p_W - r_WB) * -C_WB;

  p_W
  r_WB

  neg_C_CW = C_CB * -C_BW
  S = skew(p_W - r_WB)
  neg_C_WB = -C_WB

  % -- Jacobian w.r.t. camera extrinsics T_BCi
  jacs{2}(1:2, 1:3) = -1 * sqrt_info * Jh * -C_CB;
  jacs{2}(1:2, 4:6) = -1 * sqrt_info * Jh * -C_CB * skew(C_BCi * p_C) * -C_BCi;
  % -- Jacobian w.r.t. feature
  jacs{3} = -1 * sqrt_info * Jh * C_CW;
  % -- Jacobian w.r.t. camera parameters
  J_cam_params = camera.J_param(proj_params, dist_params, p_C);
  jacs{4} = -1 * sqrt_info * J_cam_params;
endfunction
