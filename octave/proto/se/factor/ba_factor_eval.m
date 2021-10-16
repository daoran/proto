function [r, jacs] = ba_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 3);

  % Setup return values
  r = zeros(2, 1);
  jacs{1} = zeros(2, 6);  % w.r.t Camera pose T_WC
  jacs{2} = zeros(2, 3);  % w.r.t Landmark r_W
  jacs{3} = zeros(2, 8);  % w.r.t Camera parameters

  % Map params
  cam_pose = params{1};
  landmark = params{2};
  camera = params{3};
  assert(isfield(camera, "project"));
  assert(isfield(camera, "J_proj"));
  assert(isfield(camera, "J_param"));

  % Project point in world frame to image plane
  T_WC = tf(cam_pose.param);
  p_W = landmark.param;
  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);
  p_C = tf_point(inv(T_WC), p_W);
  z_hat = camera.project(proj_params, dist_params, p_C);

  % Calculate residual
  sqrt_info = factor.sqrt_info;
  z = factor.z;
  r = sqrt_info * (z - z_hat);

  % Calculate Jacobians
  % -- Measurement model jacobian
  J_h = camera.J_proj(proj_params, dist_params, p_C);
  % -- Jacobian w.r.t. camera pose T_WC
  C_WC = tf_rot(T_WC);
  C_CW = C_WC';
  r_WC = tf_trans(T_WC);
  jacs{1}(1:2, 1:3) = -1 * sqrt_info * J_h * -C_CW;
  jacs{1}(1:2, 4:6) = -1 * sqrt_info * J_h * -C_CW * skew(p_W - r_WC) * -C_WC;
  % -- Jacobian w.r.t. landmark
  jacs{2} = -1 * sqrt_info * J_h * C_CW;
  % -- Jacobian w.r.t. camera parameters
  J_cam_params = camera.J_param(proj_params, dist_params, p_C);
  jacs{3} = -1 * sqrt_info * J_cam_params;
endfunction
