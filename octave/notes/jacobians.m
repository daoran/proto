addpath(genpath("."));

# Settings
step_size = 1.0e-9;
threshold = 1.0e-5;

% IMU to cam0 T_SC0
C_SC0 = euler321(deg2rad([0.0, 0.0, 90.0]));
r_SC0 = [0.05; 0.0; 0.0];
T_SC0 = tf(C_SC0, r_SC0);

% Sensor pose T_WS
g = [0.0; 0.0; -9.81];  % Gravity vector
a_m = [9.81; 0.0; 0.0];  % Accelerometer reading
% a_m = [9.2681; -0.310816; -3.14984];  % Accelerometer reading
q_WS = vecs2quat(a_m, -g);
C_WS = quat2rot(q_WS);
r_WS = [0.01; 0.02; 0.03];
T_WS = tf(C_WS, r_WS);

% Fiducial pose T_WF
rpy_WF = [randf([-1.0, 1.0]); randf([-1.0, 1.0]); randf([-1.0, 1.0])];
C_WF = euler321(deg2rad(rpy_WF));
r_WF = [randf([-1.0, 1.0]); randf([-1.0, 1.0]); randf([-1.0, 1.0])];
T_WF = tf(C_WF, r_WF);

% General
p_F = [randf([-1.0, 1.0]); randf([-1.0, 1.0]); randf([-1.0, 1.0])];
p_W = (T_WF * homogeneous(p_F))(1:3);
p_S = (tf_inv(T_WS) * homogeneous(p_W))(1:3);
p_C0 = (tf_inv(T_SC0) * homogeneous(p_S))(1:3);


function retval = check_dp_C__dtheta_SC(T_WS,
                                        T_SC0,
                                        T_WF,
                                        p_F,
                                        dp_C__dtheta_SC,
                                        step_size,
                                        threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0S = inv(T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_SC0_diff = perturb_rot(T_SC0, step_size, i);
    T_C0S_diff = inv(T_SC0_diff);
    p_C_diff = (T_C0S_diff * [p_S; 1])(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dtheta_SC", fdiff, dp_C__dtheta_SC, threshold);
endfunction

function retval = check_dp_C__dtheta_WS(T_WS,
                                        T_SC0,
                                        T_WF,
                                        p_F,
                                        dp_C__dtheta_SC,
                                        step_size,
                                        threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0S = inv(T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_WS_diff = perturb_rot(T_WS, step_size, i);
    T_SW_diff = inv(T_WS_diff);
    hp_S_diff = (T_SW_diff * [p_W; 1]);
    p_C_diff = (T_C0S * hp_S_diff)(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dtheta_SC", fdiff, dp_C__dtheta_SC, threshold);
endfunction

function retval = check_dp_C__dtheta_WF(T_WS,
                                        T_SC0,
                                        T_WF,
                                        p_F,
                                        dp_C__dtheta_WF,
                                        step_size,
                                        threshold)
  T_C0S = inv(T_SC0);
  T_C0W = inv(T_SC0) * inv(T_WS);
  p_C = (T_C0W * T_WF * homogeneous(p_F))(1:3);
  % p_S = (T_SC0 * homogeneous(p_C))(1:3);
  % p_W = (T_WS * homogeneous(p_S))(1:3);
  p_W = (T_WF * homogeneous(p_F))(1:3);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_WF_diff = perturb_rot(T_WF, step_size, i);
    hp_W_diff = (T_WF_diff * [p_F; 1]);
    p_C_diff = (T_C0W * hp_W_diff)(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor
  fdiff - dp_C__dtheta_WF

  retval = check_jacobian("dp_C__dtheta_WF", fdiff, dp_C__dtheta_WF, threshold);
endfunction

C_SC0 = tf_rot(T_SC0);
r_SC0 = tf_trans(T_SC0);
dp_C0__dtheta_SC = C_SC0' * skew(p_S - r_SC0);
retval = check_dp_C__dtheta_SC(T_WS, T_SC0, T_WF, p_F, dp_C0__dtheta_SC, step_size, threshold);
assert(retval == 0);

C_WS = tf_rot(T_WS);
r_WS = tf_trans(T_WS);
C_C0S = inv(tf_rot(T_SC0));
dp_C0__dtheta_WS = C_C0S * C_WS' * skew(p_W - r_WS);
retval = check_dp_C__dtheta_WS(T_WS, T_SC0, T_WF, p_F, dp_C0__dtheta_WS, step_size, threshold);
assert(retval == 0);


C_WF = tf_rot(T_WF);
r_WF = tf_trans(T_WF);

C_C0S = tf_rot(T_SC0)';
C_SW = tf_rot(T_WS)';
C_C0W = C_C0S * C_SW;

dp_W__dtheta_WF = -skew(C_WF * p_F);
dp_C0__dp_W = C_C0W;
dp_C0__dtheta_WF = dp_C0__dp_W * dp_W__dtheta_WF;
check_dp_C__dtheta_WF(T_WS, T_SC0, T_WF, p_F, dp_C0__dtheta_WF, step_size, threshold);
assert(retval == 0);
