% pkg install -forge symbolic;
pkg load symbolic;

% Setup symbols
syms k1 k2 p1 p2;
syms x y;

x2 = x * x;
y2 = y * y;
xy = x * y;
r2 = x2 + y2;
r4 = r2 * r2;

% Radial distortion
radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
x_dash = x * radial_factor;
y_dash = y * radial_factor;

% Tangential distortion
xy = x * y;
x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

% Jacobian of radtan w.r.t input x, y
J = jacobian([x_ddash, y_ddash], [x, y]);
jac_func = function_handle(J);
char(J(1, 1))
char(J(2, 1))
char(J(1, 2))
char(J(2, 2))
