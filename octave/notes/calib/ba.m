addpath(genpath("prototype"));

function z_hat = project(q_WC, C_WC, p_W, K)
  # Project landmark to image plane
  R_WC = quat2rot(q_WC);

  T_WC = transform(R_WC, C_WC);
  T_CW = tf_inv(T_WC);
  R_CW = T_CW(1:3, 1:3);
  t_CW = T_CW(1:3, 4);

  P = K * [R_CW, t_CW];
  x = P * [p_W; 1.0];

  z_hat = zeros(2, 1);
  z_hat(1) = x(1) / x(3);
  z_hat(2) = x(2) / x(3);
endfunction

function J_R = jacobian_dR_dq(q_WC)
  qx = q_WC(1);
  qy = q_WC(2);
  qz = q_WC(3);
  qw = q_WC(4);

  J_R11 = [2*qx, -2*qy, -2*qz, 2*qw];
  J_R12 = [2, 2, 2, -2];
  J_R13 = [2, 2, 2, 2];

  J_R21 = [2, 2, 2, 2];
  J_R22 = [-2*qx, 2*qy, -2*qz, 2*qw];
  J_R23 = [-2, 2, 2, 2];

  J_R31 = [2, 2, 2, -2];
  J_R32 = [2, 2, 2, 2];
  J_R33 = [-2*qx, -2*qy, 2*qz, 2*qw];

  J_R = [J_R11; J_R12; J_R13; J_R21; J_R22; J_R23; J_R31; J_R32; J_R33;];
endfunction

function J = jacobian(theta, K)
  nb_measurements = rows(theta) / 10.0;

  J_rows = nb_measurements * 2;
  J_cols = nb_measurements * 7 + 3;
  J = zeros(J_rows, J_cols);
  for i = 1:nb_measurements
    start = ((i - 1) * 10) + 1;
    # Extract the i-th q_WC, R_WC, C_WC and p_W from theta
    q_WC = theta(start:start+3, 1);
    R_WC = quat2rot(q_WC);
    C_WC = theta(start+4:start+6, 1);
    p_W = theta(start+7:start+9, 1);

    # Project landmark to image plane
    T_CW = tf_inv(transform(quat2rot(q_WC), C_WC));
    R_CW = T_CW(1:3, 1:3);
    C_CW = T_CW(1:3, 4);
    P = K * [R_CW, C_CW];
    x = P * [p_W; 1.0];

    # Build local jacobians
    dg_dh = [1 / x(3), 0, x(1) / x(3)**2; ...
             0, 1 / x(3), x(2) / x(3)**2];
    fx = K(1, 1);
    fy = K(2, 2);
    cx = K(1, 3);
    cy = K(2, 3);
    dh_dR = [(fx * (C_CW - p_W))', zeros(1, 3), (cx * (C_CW - p_W))'; ...
             zeros(1, 3), (fy * (C_CW - p_W))', (cy * (C_CW - p_W))'; ...
             zeros(1, 3), zeros(1, 3), (C_CW - p_W)'];
    dR_dq = jacobian_dR_dq(q_WC);
    dh_dC = -1 * K * R_CW;
    dh_dL = K * R_CW;

    # Add local to global jacobian
    rs = ((i - 1) * 2) + 1;
    re = rs + 1;
    cs = ((i - 1) * 7) + 1;
    ce = cs + 3;
    J(rs:re, cs:cs+3) = dg_dh * dh_dR * dR_dq;
    J(rs:re, cs+4:cs+6) = dg_dh * dh_dC;
    J(rs:re, J_cols-2:J_cols) = dg_dh * dh_dL;
  endfor
endfunction

function r = residuals(z, theta, K)
  nb_measurements = rows(z) / 2.0;
  r = [];

  for i = 1:nb_measurements
    # Calculate estimate z_hat
    start = ((i - 1) * 10) + 1;
    q_WC = theta(start:start+3, 1);
    C_WC = theta(start+4:start+6, 1);
    p_W = theta(start+7:start+9, 1);
    z_hat = project(q_WC, C_WC, p_W, K);

    # Get measurement z
    start = ((i - 1) * 2) + 1;
    z_i = z(start:start+1);

    # Calculate residual
    e = z_i - z_hat;
    r = [r; e];
  endfor
endfunction

function [z, theta] = optimization_setup(data)
  theta = [];
  z = [];

  for i = 1:columns(data.time)
    q_WC = euler2quat(data.camera_orientation(1:3, i));
    C_WC = data.camera_position(1:3, i);
    p_W = data.chessboard.corners(1:3, i);

    nb_measurements = columns(data.z_data{i});
    for j = 1:nb_measurements
      z_j = data.z_data{i}(1:2, j);
      p_W = data.landmark_data{i}(1:3, j);

      theta = [theta; q_WC; C_WC; p_W];
      z = [z; z_j];
    endfor
  endfor
endfunction


# Create chessboard
t_WT = [5; 0; 0];
R_WT = euler321([0.0, -pi / 2, 0]);
T_WT = transform(R_WT, t_WT);
chessboard = chessboard_create(T_WT);

# Create camera
t_WC = [0; 0; 0];
R_WC = euler321([deg2rad(-90.0), 0.0, deg2rad(-90.0)]);
T_WC = transform(R_WC, t_WC);
camera = camera_create(T_WC);

# Create trajectory
data = trajectory_simulate(camera, chessboard);

% rpy = [deg2rad(-90.0), 0.0, deg2rad(10.0)];
% R = euler321(rpy)
% R = quat2rot(euler2quat(rpy))


% p_W = [5; 0; 0]
% K = camera.K;

% T_WC = transform(R_WC, t_WC);
% T_CW = tf_inv(T_WC);
% R_CW = T_CW(1:3, 1:3);
% t_CW = T_CW(1:3, 4);

% x = K * R_CW * (p_W - t_CW);
% x(1) = x(1) / x(3);
% x(2) = x(2) / x(3);
% x(3) = x(3) / x(3);
% x


% # Optimize
[z, theta] = optimization_setup(data);

z = z(1:4);
theta = theta(1:20);

% Calculate the cost
cost = 0.5 * e' * e;

update_jacobian = 1;
lambda = 0.01;
nb_params = rows(theta);
cost = 0.0;
iter = 1;

% if update_jacobian == 1
	J = jacobian(theta, camera.K);
	H = J' * J; # Approximate Hessian matrix
	e = residuals(z, theta, camera.K);

	% compute the approximated Hessian matrix, J’ is the transpose of J
	if iter == 1 % the first iteration : compute the total error
		cost = 0.5 * e' * e;
  end
% end

% nb_params
% theta_rows = rows(theta);
% theta_cols = columns(theta);
% theta_rows / 10;

% Apply the damping factor to the Hessian matrix
H_rows = rows(H);
H_cols = columns(H);
H_lm = H + (lambda * eye(H_rows, H_cols));

size(J)

% Compute the updated parameters
% dtheta = -inv(H_lm) * (J' * e(:));
% theta_lm = theta + dtheta;

% % If the total distance error of the updated parameters is less than the
% % previous one then makes the updated parameters to be the current parameters
% % and decreases the value of the damping factor
% if e_lm < e
% 	lambda = lambda / 10;
% 	a_est = a_lm;
% 	b_est = b_lm;
% 	e = e_lm;
% 	% disp(e);
% 	update_jacobian = 1;
% else
% 	% Increases the value of the damping factor
% 	update_jacobian = 0;
% 	lambda = lamda * 10;
% end

% Solve!
% EWE = E' * (W \ E);
% RCOND = rcond(EWE);
% dx_star =  (EWE) \ (-E' * (W \ errorVec));
% xEst = xEst + dx_star;
% Jderiv = abs((Jnew - Jprev) / Jnew);
% Jprev = Jnew;
% if Jderiv < 0.01
%   break;
% else
%   alphaBar = xEst(1);
%   betaBar = xEst(2);
%   rhoBar = xEst(3);
% end

# Plot trajectory
% figure;
% hold on;
% trajectory_plot(data);
% xlim([0 8]);
% ylim([-4 4]);
% zlim([-4 4]);
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% pause;
