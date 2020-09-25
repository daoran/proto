function [r, jacobians] = cam_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 4);
  assert(isfield(params{4}, "project"));
  assert(isfield(params{4}, "J_proj"));
  assert(isfield(params{4}, "J_param"));

  % Setup return values
  r = zeros(2, 1);
  jacobians{1} = zeros(2, 6);  % w.r.t Sensor pose T_WS
  jacobians{2} = zeros(2, 6);  % w.r.t Sensor-camera pose T_SC
  jacobians{3} = zeros(2, 3);  % w.r.t Landmark r_W
  jacobians{4} = zeros(2, 8);  % w.r.t Camera parameters

  % Map params
  sensor_pose = params{1};
  sensor_cam_pose = params{2};
  landmark = params{3};
  camera = params{4};

  T_WS = tf(sensor_pose.param);
  T_SC = tf(sensor_cam_pose.param);
  p_W = landmark.param;

  % Project point in world frame to image plane
  p_C = tf_point(inv(T_WS * T_SC), p_W);
  z_hat = camera.project(proj_params, dist_params, p_C);

  % Calculate residual
  sqrt_info = chol(factor.covar);
  z = factor.z;
  r = sqrt_info * (z - z_hat);

  % Calculate Jacobians
  % -- Measurement model jacobian
  J_h = camera.J_proj(proj_params, dist_params, p_C);
  % -- Jacobian w.r.t. sensor pose T_WS
  C_WS = tf_rot(T_WS);
  C_SW = C_WS';
  r_WS = tf_trans(T_WS);
  jacobians{1}(1:2, 1:3) = -1 * sqrt_info * J_h * C_CS * C_SW * skew(p_W - r_WS);
  jacobians{1}(1:2, 4:6) = -1 * sqrt_info * J_h * C_CS * -C_SW;
  % -- Jacobian w.r.t. sensor camera pose T_SC
  jacobians{2}(1:2, 1:3) = -1 * sqrt_info * J_h * C_CS * skew(C_SC * p_C);
  jacobians{2}(1:2, 4:6) = -1 * sqrt_info * J_h * -C_CS;
  % -- Jacobian w.r.t. landmark
  jacobians{3} = -1 * sqrt_info * J_h * C_CW;
  % -- Jacobian w.r.t. camera parameters
  J_cam_params = camera.J_param(proj_params, dist_params, p_C);
  jacobians{4} = -1 * sqrt_info * J_cam_params;
endfunction
