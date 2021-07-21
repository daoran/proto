function [r, jacs] = imu_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 4);

  % Map params
  pose_i = params{1};
  sb_i = params{2};
  pose_j = params{3};
  sb_j= params{4};

  % Timestep i
  T_i = tf(pose_i.param);
  r_i = tf_trans(T_i);
  C_i = tf_rot(T_i);
  q_i = tf_quat(T_i);
  v_i = sb_i.param(1:3);
  ba_i = sb_i.param(4:6);
  bg_i = sb_i.param(7:9);

  % Timestep j
  T_j = tf(pose_j.param);
  r_j = tf_trans(T_j);
  C_j = tf_rot(T_j);
  q_j = tf_quat(T_j);
  v_j = sb_j.param(1:3);
  ba_j = sb_j.param(4:6);
  bg_j = sb_j.param(7:9);

  % Correct the relative position, velocity and orientation
  % -- Extract jacobians from error-state jacobian
  dr_dba = factor.state_F(1:3, 10:12);
  dr_dbg = factor.state_F(1:3, 13:15);
  dv_dba = factor.state_F(4:6, 10:12);
  dv_dbg = factor.state_F(4:6, 13:15);
  dq_dbg = factor.state_F(7:9, 13:15);
  dba = ba_i - factor.ba;
  dbg = bg_i - factor.bg;
  % -- Correct the relative position, velocity and rotation
  dr = factor.dr + dr_dba * dba + dr_dbg * dbg;
  dv = factor.dv + dv_dba * dba + dv_dbg * dbg;
  dq = quat_mul(quat_delta(dq_dbg * dbg), rot2quat(factor.dC));
  dC_ = factor.dC;
  dC = factor.dC * Exp(dq_dbg * dbg);

  % Form residuals
  g = factor.g;
  Dt = factor.Dt;
  Dt_sq = Dt * Dt;
  err_pos = (C_i' * ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq))) - dr;
  err_vel = (C_i' * ((v_j - v_i) + (g * Dt))) - dv;
  err_rot = (2 * quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i), q_j)))(2:4);
  % err_rot = Log(dC' * C_i' * C_j);
  err_ba = zeros(3, 1);
  err_bg = zeros(3, 1);
  r = [err_pos; err_vel; err_rot; err_ba; err_bg];

  % Form jacobians
  jacs{1} = zeros(15, 6);  % residuals w.r.t pose i
  jacs{2} = zeros(15, 9);  % residuals w.r.t speed and biase i
  jacs{3} = zeros(15, 6);  % residuals w.r.t pose j
  jacs{4} = zeros(15, 9);  % residuals w.r.t speed and biase j

  % -- Jacobian w.r.t. pose i
  jacs{1}(1:3, 1:3) = -C_i';                                                       % dr w.r.t r_i
  jacs{1}(1:3, 4:6) = skew(C_i' * ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq))); % dr w.r.t C_i
  jacs{1}(4:6, 4:6) = skew(C_i' * ((v_j - v_i) + (g * Dt)));                       % dv w.r.t C_i
  jacs{1}(7:9, 4:6) = -Jr_inv(err_rot) * (C_j' * C_i);                             % dtheta w.r.t C_i

  % -- Jacobian w.r.t. speed and biases i
  jacs{2}(1:3, 1:3) = -C_i' * Dt;  % dr w.r.t v_i
  jacs{2}(1:3, 4:6) = -dr_dba;     % dr w.r.t ba
  jacs{2}(1:3, 7:9) = -dr_dbg;     % dr w.r.t bg
  jacs{2}(4:6, 1:3) = -C_i';       % dv w.r.t v_i
  jacs{2}(4:6, 4:6) = -dv_dba;     % dv w.r.t ba
  jacs{2}(4:6, 7:9) = -dv_dbg;     % dv w.r.t bg
  % jacs{2}(7:9, 7:9) = -Jr_inv(err_rot) * Exp(err_rot)' * Jr(dq_dbg * dbg) * dq_dbg;  % dtheta w.r.t bg_i
  jacs{2}(7:9, 7:9) = -quat_left(C_j' * C_i * dC_)(2:4, 2:4) * dq_dbg;

  % -- Jacobian w.r.t. pose j
  jacs{3}(1:3, 1:3) = C_i';                                             % dr w.r.t r_j
  jacs{3}(7:9, 4:6) = quat_left(rot2quat(dC' * C_i' * C_j))(2:4, 2:4);  % dtheta w.r.t C_j

  % -- Jacobian w.r.t. sb j
  jacs{4}(4:6, 1:3) = C_i';       % dv w.r.t v_j
endfunction
