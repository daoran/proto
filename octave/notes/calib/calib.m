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
r_SC0 = tf_trans(T_SC0);
% -- de__dtheta_SC0
dp_C0__dtheta_SC0 = C_SC0' * skew(p_S - r_SC0);
de__dtheta_SC0 = de__dzhat * dzhat__dp_C0 * dp_C0__dtheta_SC0;
% -- de__dr_SC0
C_C0S = inv(tf_rot(T_SC0));
dp_C0__dr_C0S = eye(3);
dr_C0S__dr_SC0 = -C_C0S;
de__dr_SC0 = de__dzhat * dzhat__dp_C0 * dp_C0__dr_C0S * dr_C0S__dr_SC0;

% Jacobian w.r.t. sensor pose
C_WS = tf_rot(T_WS);
r_WS = tf_trans(T_WS);
C_SW = C_WS';
C_C0W = tf_rot(T_SC0)' * tf_rot(T_WS)';
C_C0S = tf_rot(T_SC0)';
% -- de__dtheta_WS
dp_C0__dp_S = C_C0S;
dp_S__dtheta_WS = C_WS' * skew(p_W - r_WS);
de__dtheta_WS = de__dzhat * dzhat__dp_C0 * dp_C0__dp_S * dp_S__dtheta_WS;
% -- de__dr_WS
dp_S__dr_SW = eye(3);
dr_SW__dr_WS = -C_WS';
de__dr_WS = de__dzhat * dzhat__dp_C0 * dp_C0__dp_S * dp_S__dr_SW * dr_SW__dr_WS;

% Jacobian w.r.t. fiducial pose
C_WF = tf_rot(T_WF);
C_C0W = tf_rot(T_SC0)' * tf_rot(T_WS)';
dp_C0__dp_W = C_C0W;
% -- de__dtheta_WF
dp_W__dtheta_WF = -skew(C_WF * p_F);
de__dtheta_WF = de__dzhat * dzhat__dp_C0 * dp_C0__dp_W * dp_W__dtheta_WF;
% -- de__dr_WF
dp_W__dr_WF = eye(3);
de__dr_WF = de__dzhat * dzhat__dp_C0 * dp_C0__dp_W * dp_W__dr_WF;

% Check jacobians
retval = 0;
retval += check_de__dtheta_SC0(T_WS, T_SC0, T_WF, p_F, de__dtheta_SC0, step_size, threshold);
retval += check_de__dtheta_WS(T_WS, T_SC0, T_WF, p_F, de__dtheta_WS, step_size, threshold);
retval += check_de__dtheta_WF(T_WS, T_SC0, T_WF, p_F, de__dtheta_WF, step_size, threshold);
retval += check_de__dr_SC0(T_WS, T_SC0, T_WF, p_F, de__dr_SC0, step_size, threshold);
retval += check_de__dr_WS(T_WS, T_SC0, T_WF, p_F, de__dr_WS, step_size, threshold);
retval += check_de__dr_WF(T_WS, T_SC0, T_WF, p_F, de__dr_WF, step_size, threshold);

% Summary
if retval != 0
  printf("Test failed!\n");
else
  printf("Test passed!\n");
endif
