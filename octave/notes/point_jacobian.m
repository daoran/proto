addpath(genpath("prototype"));

p_W = [unifrnd(-1, 1); unifrnd(-1, 1); unifrnd(-1, 1)];
% hp_W = homogeneous(p_W)

R_WF = euler321([0, 0, 0]);
t_WF = [0; 0; 0];
T_WF = transform(R_WF, t_WF);

R_WS = euler321([0, 0, 0]);
t_WS = [0; 0; 0];
T_WS = transform(R_WS, t_WS);

R_SC0 = euler321([0, 0, 0]);
t_SC0 = [0; 0; 0];
T_SC0 = transform(R_SC0, t_SC0);

R_C0W = euler321([unifrnd(-0.5, 0.5), unifrnd(-0.5, 0.5), unifrnd(-0.5, 0.5)]);
t_C0W = [unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05)];

L_W = [0.1; 0.1; 2.0];
L_C = R_C0W * L_W + t_C0W;
z = [L_C(1) / L_C(3); L_C(2) / L_C(3)];

dL_C__dL_W = R_C0W;

dh__dL_C = zeros(2, 3);
dh__dL_C(1, 1) = 1.0;
dh__dL_C(2, 2) = 1.0;
dh__dL_C(1, 3) = -(L_C(1) / L_C(3));
dh__dL_C(2, 3) = -(L_C(2) / L_C(3));
dh__dL_C = 1 / L_C(3) * dh__dL_C;

% Perform numerical diff to obtain finite difference
step_size = 1e-6;
eps = eye(3) * step_size;
% finite_diff = zeros(3, 3);
finite_diff = zeros(2, 3);
for i = 1:3
  L_W_diff = L_W + eps(1:3, i);
  L_C_diff = (R_C0W * L_W_diff) + t_C0W;
  % finite_diff(1:3, i) = (L_C_diff - L_C) / step_size;

  z_hat = [L_C_diff(1) / L_C_diff(3); L_C_diff(2) / L_C_diff(3)];
  finite_diff(1:2, i) = (z_hat - z) / step_size;
endfor

% dL_C__dL_W
% finite_diff
% finite_diff - dL_C__dL_W

dh__dL_W = dh__dL_C * dL_C__dL_W
finite_diff
finite_diff - dh__dL_W
