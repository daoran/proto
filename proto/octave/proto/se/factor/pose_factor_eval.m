function [r, jacs] = pose_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 1);

  % Setup return values
  r = zeros(6, 1);
  jacobians{1} = zeros(6, 6);

  % Measured pose
  T_meas = factor.pose;
  q_meas = tf_quat(T_meas);
  r_meas = tf_trans(T_meas);

  % Estimated pose
  pose_est = params{1};
  T_est = tf(pose_est.param);
  q_est = tf_quat(T_est);
  r_est = tf_trans(T_est);

  % Form residuals (pose - pose_est)
  dr = r_meas - r_est;
  dq = quat_mul(quat_inv(q_meas), q_est);
  dtheta = 2 * dq(2:4);
  r = factor.sqrt_info * [dr; dtheta];

  % Form jacobians
  jacs{1} = zeros(6, 6);
  jacs{1}(1:3, 1:3) = -eye(3);
  jacs{1}(4:6, 4:6) = quat_left(dq)(2:4, 2:4);
  jacs{1} = factor.sqrt_info * jacs{1};
endfunction
