addpath(genpath("."));

# Settings
step_size = 1.0e-9;
threshold = 1.0e-5;

% IMU to cam0 T_SC0
C_SC0 = euler321([deg2rad(0.0), deg2rad(0.0), deg2rad(90.0)]);
r_SC0 = [0.05; 0; 0.0];
T_SC0 = transform(C_SC0, r_SC0);

% Sensor pose T_WS
g = [0; 0; -9.81];  % Gravity vector
a_m = [9.81; 0; 0];  % Accelerometer reading
% a_m = [9.2681; -0.310816; -3.14984];  % Accelerometer reading
q_WS = vecs2quat(a_m, -g);
C_WS = quat2rot(q_WS);
r_WS = [0; 0; 0];
T_WS = transform(C_WS, r_WS);

% Fiducial pose T_WF
rpy_WF = [deg2rad(0.1); deg2rad(0.2); deg2rad(0.1)];
C_WF = euler321(rpy_WF);
r_WF = [-1.0; 0; 0];
T_WF = transform(C_WF, r_WF);

% General
% p_F = [unifrnd(-0.5, 0.5); unifrnd(0.0, 0.5); unifrnd(-0.5, 0.5)];
p_F = [0; 0; 0.05];
p_W = (T_WF * homogeneous(p_F))(1:3);
p_S = (tf_inv(T_WS) * homogeneous(p_W))(1:3);
p_C0 = (tf_inv(T_SC0) * homogeneous(p_S))(1:3);

% Jacobian of residual w.r.t. zhat
de__dzhat = -1;

% Jacobian of zhat w.r.t point in camera frame
dzhat__dp_C0 = zeros(2, 3);
dzhat__dp_C0(1, 1) = 1.0 / p_C0(3);
dzhat__dp_C0(2, 2) = 1.0 / p_C0(3);
dzhat__dp_C0(1, 3) = -(p_C0(1) / p_C0(3)**2);
dzhat__dp_C0(2, 3) = -(p_C0(2) / p_C0(3)**2);

% Jacobian w.r.t. sensor cam extrinsics
C_SC0 = tf_rot(T_SC0);
C_C0S = C_SC0';
dtheta_C0S__dtheta_SC0 = -C_C0S;
dp_C0__dtheta_C0S = -skew(C_C0S * p_S);
dp_S__dtheta_SC0 = -skew(C_SC0 * p_C0);
dp_C0__dp_S = C_C0S;

dp_C0__dtheta_SC0 = dp_C0__dtheta_C0S * dtheta_C0S__dtheta_SC0;
dzhat__dtheta_SC0 = dzhat__dp_C0 * dp_C0__dtheta_SC0;

% Finite difference
fdiff = zeros(3, 3);
p_C0 = (tf_inv(T_SC0) * tf_inv(T_WS) * T_WF * homogeneous(p_F))(1:3);
for i = 1:3
  T_SC0_diff = perturb_rot(T_SC0, step_size, i);
  p_C0_prime = (tf_inv(T_SC0_diff) * tf_inv(T_WS) * T_WF * homogeneous(p_F))(1:3);
  fdiff(1:3, i) = (p_C0_prime - p_C0) / step_size;
endfor
retval = check_jacobian("dp_C0__dtheta_SC0", fdiff, dp_C0__dtheta_SC0, threshold);


fdiff = zeros(2, 3);
zhat = h(T_WS, T_SC0, T_WF, p_F);
for i = 1:3
  T_SC0_diff = perturb_rot(T_SC0, step_size, i);
  zhat_prime = h(T_WS, T_SC0_diff, T_WF, p_F);
  fdiff(1:2, i) = (zhat_prime - zhat) / step_size;
endfor
retval = check_jacobian("dzhat__dtheta_SC0", fdiff, dzhat__dtheta_SC0, threshold);
