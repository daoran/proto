addpath(genpath("proto"));
pkg load symbolic;

% Setup symbols
syms k1 k2 k3 k4;
syms x y;

r = sqrt(x * x + y * y);
th = atan(r);
th2 = th * th;
th4 = th2 * th2;
th6 = th4 * th2;
th8 = th4 * th4;
thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
s = thd / r;
x_dash = s * x;
y_dash = s * y;

% Jacobian of radtan w.r.t input x, y
J = jacobian([x_dash, y_dash], [x, y]);
x_func = function_handle(x_dash);
y_func = function_handle(y_dash);
jac_func = function_handle(J);
% char(J(1, 1))
% char(J(2, 1))
% char(J(1, 2))
% char(J(2, 2))

% Setup equi coefficients
k1 = unifrnd(-0.1, 0.1);
k2 = unifrnd(-0.01, 0.01);
k3 = unifrnd(-0.001, 0.001);
k4 = unifrnd(-0.001, 0.001);

% Form and distort point
point = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
point0 = [x_func(k1, k2, k3, k4, point(1), point(2));
          y_func(k1, k2, k3, k4, point(1), point(2))];
J = jac_func(k1, k2, k3, k4, point(1), point(2));

% Perform numerical diff to obtain finite difference
step_size = 1e-6;
eps = eye(2) * step_size;
finite_diff = zeros(2, 2);
for i = 1:2
  point1 = point + eps(1:2, i);
  point1 = [x_func(k1, k2, k3, k4, point1(1), point1(2));
            y_func(k1, k2, k3, k4, point1(1), point1(2))];
  finite_diff(1:2, i) = (point1 - point0) / step_size;
endfor

threshold = 1e-6;
check_jacobian("J", finite_diff, J, threshold)
