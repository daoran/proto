addpath(genpath("prototype"));

# Settings
step_size = 1e-8;
threshold = 1e-5;

% IMU to cam0 T_SC0
R_SC0 = euler321([deg2rad(-90.0), 0.0, deg2rad(-90.0)]);
% R_SC0 = [0.0, -1.0, 0.0;
%          1.0, 0.0, 0.0;
%          0.0, 0.0, 1.0];
t_SC0 = [0.0; 0; 0.05];
T_SC0 = transform(R_SC0, t_SC0);

% Sensor pose T_WS
g = [0; 0; -9.81];  % Gravity vector
% a_m = [9.81; 0; 0];  % Accelerometer reading
a_m = [9.2681; -0.310816; -3.14984];  % Accelerometer reading
q_WS = vecs2quat(a_m, -g);
R_WS = quat2rot(q_WS);
t_WS = [0; 0; 0];
T_WS = transform(R_WS, t_WS);

% Fiducial pose T_WF
rpy_WF = [deg2rad(0.0); deg2rad(0.0); deg2rad(0.0)];
R_WF = euler321(rpy_WF);
t_WF = [10.0; 0; 0];
T_WF = transform(R_WF, t_WF);


% % Plot
% figure(1);
% hold on;
% grid on;
% view(3);
% draw_frame(T_WS(1:3, 1:3), 0.1);
% draw_camera(T_WS * T_SC0, scale=0.05, 'r');
% xlabel("x");
% ylabel("y");
% zlabel("z");
% axis('equal');
% ginput();


% Fiducial point
p_F = [unifrnd(-0.5, 0.5); unifrnd(-0.5, 0.5); unifrnd(-0.5, 0.5)];
% -- Point projected in camera and sensor frame
p_W = (T_WF * homogeneous(p_F))(1:3);
p_S = (inv(T_WS) * T_WF * homogeneous(p_F))(1:3);
p_C = (inv(T_SC0) * inv(T_WS) * T_WF * homogeneous(p_F))(1:3);


% Jacobian of h() w.r.t point in camera frame
dh__dp_C = zeros(2, 3);
dh__dp_C(1, 1) = 1.0 / p_C(3);
dh__dp_C(2, 2) = 1.0 / p_C(3);
dh__dp_C(1, 3) = -(p_C(1) / p_C(3)**2);
dh__dp_C(2, 3) = -(p_C(2) / p_C(3)**2);


% Jacobian w.r.t. sensor pose
C_C0W = T_SC0(1:3, 1:3)' * T_WS(1:3, 1:3)';
dp_C__dp_W = C_C0W;

dp_W__dtheta_WS = -skew(T_WS(1:3, 1:3) * p_S);
dh__dtheta_WS = dh__dp_C * dp_C__dp_W * dp_W__dtheta_WS;

dp_W__dr_WS = eye(3);
dh__dr_WS = dh__dp_C * dp_C__dp_W * dp_W__dr_WS;


% Jacobian w.r.t. sensor-camera extrinsics
C_SC0 = T_SC0(1:3, 1:3);
dp_C__dp_S = C_SC0';

dp_S__dtheta_SC = -skew(C_SC0 * p_C);
dp_S__dr_SC = eye(3);

dh__dtheta_SC = dh__dp_C * dp_C__dp_S * dp_S__dtheta_SC;
dh__dr_SC = dh__dp_C * dp_C__dp_S * dp_S__dr_SC;


% Jacobian w.r.t. fiducial pose
C_WF = T_WF(1:3, 1:3);
dp_C__dp_W = T_SC0(1:3, 1:3)' * T_WS(1:3, 1:3)';

dp_W__dtheta_WF = -skew(C_WF * p_F);
dh__dtheta_WF = -1 * dh__dp_C * dp_C__dp_W * dp_W__dtheta_WF;

dp_W__dr_WF = eye(3);
dh__dr_WF = -1 * dh__dp_C * dp_C__dp_W * dp_W__dr_WF;


function retval = check_jacobian(jac_name, fdiff, jac, threshold)
  delta = abs(sum((fdiff - jac)(:)));
  if (delta > threshold)
    retval = -1;
    printf("Check [%s] failed!\n", jac_name);
    fdiff_minus_jac = fdiff - jac
    delta
    printf("----------------------------------------\n");
  else
    printf("Check [%s] passed!\n", jac_name);
    retval = 0;
  endif
endfunction

% Measurement function h(): Projects point from fiducial frame to image frame
function z = h(T_WS, T_SC0, T_WF, p_F)
  % Project point in fiducial frame to image plane
  p_C = (inv(T_SC0) * inv(T_WS) * T_WF * homogeneous(p_F))(1:3);
  z = [p_C(1) / p_C(3); p_C(2) / p_C(3)];
endfunction

function retval = check_dp_C__dp_W(T_WS, T_SC0, T_WF, p_F, dp_C__dp_W, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0W = inv(T_WS * T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    p_W_diff = p_W + step(1:3, i);
    p_C_diff = (T_C0W * [p_W_diff; 1])(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dp_W", fdiff, dp_C__dp_W, threshold);
endfunction

function retval = check_dp_C__dp_S(T_WS, T_SC0, T_WF, p_F, dp_C__dp_S, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0S = inv(T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    p_S_diff = p_S + step(1:3, i);
    p_C_diff = (T_C0S * [p_S_diff; 1])(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dp_S", fdiff, dp_C__dp_S, threshold);
endfunction

function retval = check_dh__dp_C(T_WS, T_SC0, T_WF, p_F, dh__dp_C, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  z = [p_C(1) / p_C(3); p_C(2) / p_C(3)];

  step = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    p_C_diff = p_C + step(1:3, i);
    z_hat = [p_C_diff(1) / p_C_diff(3); p_C_diff(2) / p_C_diff(3)];
    fdiff(1:2, i) = (z_hat - z) / step_size;
  endfor

  retval = check_jacobian("dh__dp_C", fdiff, dh__dp_C, threshold);
endfunction

function retval = check_dh__dtheta_WS(T_WS, T_SC0, T_WF, p_F, dh__dtheta_WS, step_size, threshold)
  rvec = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb C_WS
    C_WS = T_WS(1:3, 1:3);
    r_WS = T_WS(1:3, 4);
    C_WS_diff = rvec2rot(rvec(1:3, i));
    C_WS_diff = C_WS_diff * C_WS;
    T_WS_diff = transform(C_WS_diff, r_WS);

    % Project to image plane and get finite diff
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS_diff, T_SC0, T_WF, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dtheta_WS", fdiff, dh__dtheta_WS, threshold);
endfunction

function retval = check_dh__dr_WS(T_WS, T_SC0, T_WF, p_F, dh__dr_WS, step_size, threshold)
  dr_WS = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb r_WS
    C_WS = T_WS(1:3, 1:3);
    r_WS = T_WS(1:3, 4);

    r_WS_diff = r_WS + dr_WS(1:3, i);
    T_WS_diff = transform(C_WS, r_WS_diff);

    % Project to image plane and get finite diff
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS_diff, T_SC0, T_WF, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dr_WS", fdiff, dh__dr_WS, threshold);
endfunction

function retval = check_dh__dtheta_SC0(T_WS, T_SC0, T_WF, p_F, dh__dtheta_SC0, step_size, threshold)
  rvec = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb C_SC0
    C_SC0 = T_SC0(1:3, 1:3);
    r_SC0 = T_SC0(1:3, 4);
    C_SC0_diff = rvec2rot(rvec(1:3, i));
    C_SC0_diff = C_SC0_diff * C_SC0;
    T_SC0_diff = transform(C_SC0_diff, r_SC0);

    % Project to image plane and get finite diff
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS, T_SC0_diff, T_WF, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dtheta_SC0", fdiff, dh__dtheta_SC0, threshold);
endfunction

function retval = check_dh__dr_SC0(T_WS, T_SC0, T_WF, p_F, dh__dr_SC0, step_size, threshold)
  dr_SC0 = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb r_SC0
    C_SC0 = T_SC0(1:3, 1:3);
    r_SC0 = T_SC0(1:3, 4);
    r_SC0_diff = r_SC0 + dr_SC0(1:3, i);
    T_SC0_diff = transform(C_SC0, r_SC0_diff);

    % Project to image plane and get finite diff
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS, T_SC0_diff, T_WF, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dr_SC0", fdiff, dh__dr_SC0, threshold);
endfunction

function retval = check_dh__dtheta_WF(T_WS, T_SC0, T_WF, p_F, dh__dtheta_WF, step_size, threshold)
  rvec = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb C_WF
    C_WF = T_WF(1:3, 1:3);
    r_WF = T_WF(1:3, 4);
    C_WF_diff = rvec2rot(rvec(1:3, i));
    C_WF_diff = C_WF_diff * C_WF;
    T_WF_diff = transform(C_WF_diff, r_WF);

    % Project to image plane and get finite diff
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS, T_SC0, T_WF_diff, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dtheta_WF", fdiff, dh__dtheta_WF, threshold);
endfunction

function retval = check_dh__dr_WF(T_WS, T_SC0, T_WF, p_F, dh__dr_WF, step_size, threshold)
  dr_WF = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    % Perturb r_WF
    C_WF = T_WF(1:3, 1:3);
    r_WF = T_WF(1:3, 4);
    r_WF_diff = r_WF + dr_WF(1:3, i);
    T_WF_diff = transform(C_WF, r_WF_diff);

    % Project to image plane
    z = h(T_WS, T_SC0, T_WF, p_F);
    z_hat = h(T_WS, T_SC0, T_WF_diff, p_F);
    fdiff(1:2, i) = (z - z_hat) / step_size;
  endfor

  retval = check_jacobian("dh__dr_WF", fdiff, dh__dr_WF, threshold);
endfunction

% Check jacobians
retval = 0;
retval += check_dp_C__dp_W(T_WS, T_SC0, T_WF, p_F, dp_C__dp_W, step_size, threshold);
retval += check_dp_C__dp_S(T_WS, T_SC0, T_WF, p_F, dp_C__dp_S, step_size, threshold);
retval += check_dh__dp_C(T_WS, T_SC0, T_WF, p_F, dh__dp_C, step_size, threshold);
% -- Jacobian w.r.t sensor pose: T_WS
retval += check_dh__dtheta_WS(T_WS, T_SC0, T_WF, p_F, dh__dtheta_WS, step_size, threshold);
retval += check_dh__dr_WS(T_WS, T_SC0, T_WF, p_F, dh__dr_WS, step_size, threshold);
% -- Jacobian w.r.t sensor camera extrinsics: T_SC0
retval += check_dh__dtheta_SC0(T_WS, T_SC0, T_WF, p_F, dh__dtheta_SC, step_size, threshold);
retval += check_dh__dr_SC0(T_WS, T_SC0, T_WF, p_F, dh__dr_SC, step_size, threshold);
% -- Jacobian w.r.t fiducial pose: T_WF
retval += check_dh__dtheta_WF(T_WS, T_SC0, T_WF, p_F, dh__dtheta_WF, step_size, threshold);
retval += check_dh__dr_WF(T_WS, T_SC0, T_WF, p_F, dh__dr_WF, step_size, threshold);


% Summary
if retval != 0
  printf("Test failed!\n");
else
  printf("Test passed!\n");
endif
