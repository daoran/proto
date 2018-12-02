addpath(genpath("prototype"));
pkg load symbolic;

% % Setup symbols
% syms fx fy cx cy;
% syms x y;
% syms u v;
%
% uhat = (fx * x) + cx;
% vhat = (fy * y) + cy;
% zhat = [uhat; vhat];
%
% z = [u; v];
% residual = z - zhat;
%
% J = jacobian(zhat, [fx, fy, cx, cy])
% char(J(1, 1))
% char(J(1, 2))
% char(J(1, 3))
% char(J(1, 4))
% char(J(2, 1))
% char(J(2, 2))
% char(J(2, 3))
% char(J(2, 4))


syms fx fy cx cy;
syms k1 k2 p1 p2;
syms px py pz;

p_C = [px; py; pz];

% Radial-Tangential distortion
% -- Project point
p = [p_C(1) / p_C(3); p_C(2) / p_C(3)];
x = p(1);
y = p(2);
% -- Precompute common terms
x2 = x * x;
y2 = y * y;
xy = x * y;
r2 = x2 + y2;
r4 = r2 * r2;
% -- Radial distortion
radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
x_dash = x * radial_factor;
y_dash = y * radial_factor;
% -- Tangential distortion
xy = x * y;
x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

% Scale and offset with fx fy cx cy
uhat = (fx * x_ddash) + cx;
vhat = (fy * y_ddash) + cy;
zhat = [uhat; vhat];

J = jacobian(zhat, [fx, fy, cx, cy])
char(J(1, 1))
char(J(1, 2))
char(J(1, 3))
char(J(1, 4))
char(J(2, 1))
char(J(2, 2))
char(J(2, 3))
char(J(2, 4))







% p = [0.1; 0.2;];
% px = p(1);
% py = p(2);
%
% dzhat__dpinhole = [px, 0.0, 1.0, 0.0;
%                    0.0, py, 0.0, 1.0];
% dr__dzhat = -1 * eye(2);
% dr__dpinhole = dr__dzhat * dzhat__dpinhole;
%
%
% fdiff = zeros(2, 4);
% step_size = 1.0e-8;
%
% fx = 1.1;
% fy = 1.1;
% cx = 1.1;
% cy = 1.1;
% u = (fx * px) + cx;
% v = (fy * py) + cy;
% z = [u; v];
%
% fx_diff = fx + step_size;
% uhat = (fx_diff * px) + cx;
% vhat = (fy * py) + cy;
% zhat = [uhat; vhat];
% fdiff(1:2, 1) = (zhat - z) / step_size;
%
% fy_diff = fy + step_size;
% uhat = (fx * px) + cx;
% vhat = (fy_diff * py) + cy;
% zhat = [uhat; vhat];
% fdiff(1:2, 2) = (zhat - z) / step_size;
%
% cx_diff = cx + step_size;
% uhat = (fx * px) + cx_diff;
% vhat = (fy * py) + cy;
% zhat = [uhat; vhat];
% fdiff(1:2, 3) = (zhat - z) / step_size;
%
% cy_diff = cy + step_size;
% uhat = (fx * px) + cx;
% vhat = (fy * py) + cy_diff;
% zhat = [uhat; vhat];
% fdiff(1:2, 4) = (zhat - z) / step_size;
%
% fdiff
% dzhat__dpinhole
