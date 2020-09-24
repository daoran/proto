function [r, jacobians] = ba_factor_eval(factor, params)
  assert(isfield(factor, "param_ids"));

  % Setup return values
  r = zeros(2, 1);
  jacobians{1} = zeros(2, 6);  % w.r.t Camera pose T_WC
  jacobians{2} = zeros(2, 3);  % w.r.t Landmark r_W
  jacobians{3} = zeros(2, 8);  % w.r.t Camera parameters

  % Map params
  cam_pose = params{1};
  landmark = params{2};
  cam_params = params{3};

  T_WC = tf(cam_pose.param);
  p_W = landmark.param;
  proj_params = cam_params.param(1:4);
  dist_params = cam_params.param(5:8);

  % Project point in world frame to image plane
  p_C = tf_point(inv(T_WC), p_W);
  z_hat = pinhole_radtan4_project(proj_params, dist_params, p_C);

  % Calculate residual
  sqrt_info = chol(factor.covar);
  z = factor.z;
  r = sqrt_info * (z - z_hat);

  % Calculate Jacobians
  fx = proj_params(1);
  fy = proj_params(2);

  k1 = dist_params(1);
  k2 = dist_params(2);
  p1 = dist_params(3);
  p2 = dist_params(4);

  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];       % Project 3D point
  x_dist = radtan4_distort(k1, k2, p1, p2, x);  % Distort point

  % -- Measurement model jacobian
  J_proj = [1 / p_C(3), 0, -p_C(1) / p_C(3)**2;
            0, 1 / p_C(3), -p_C(2) / p_C(3)**2];
  J_dist = radtan4_point_jacobian(k1, k2, p1, p2, x);
  J_point = [fx, 0; 0, fy];
  J_h = J_point * J_dist * J_proj;

  % -- Jacobian w.r.t. camera pose T_WC
  C_WC = tf_rot(T_WC);
  C_CW = C_WC';
  r_WC = tf_trans(T_WC);
  jacobians{1}(1:2, 1:3) = -1 * sqrt_info * J_h * C_CW * skew(p_W - r_WC);
  jacobians{1}(1:2, 4:6) = -1 * sqrt_info * J_h * -C_CW;

  % -- Jacobian w.r.t. landmark
  jacobians{2} = -1 * sqrt_info * J_h * C_CW;

  % -- Jacobian w.r.t. camera parameters
  J_cam_params = zeros(2, 8);
  J_cam_params(1:2, 1:4) = [x_dist(1), 0, 1, 0;
                            0, x_dist(2), 0, 1];
  J_dist = radtan4_params_jacobian(k1, k2, p1, p2, x);
  J_cam_params(1:2, 5:8) = J_point * J_dist;
  jacobians{3} = -1 * sqrt_info * J_cam_params;
endfunction
