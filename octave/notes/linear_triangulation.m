% pkg load symbolic;
%
% syms P11 P12 P13 P14;
% syms P21 P22 P23 P24;
% syms P31 P32 P33 P34;
% syms Xx Xy Xz Xw;
% syms x y z;
%
% P = [P11, P12, P13, P14;
%      P21, P22, P23, P24;
%      P31, P32, P33, P34];
% X = [Xx; Xy; Xz; 1.0];
% z = [x; y; 1.0];

% cross(z, (P * X))

% function linear_triangulation(z, z_dash, P, P_dash)
%   x = z(1);
%   y = z(2);
%   x_dash = z_dash(1);
%   y_dash = z_dash(2);
%
%   P1T = P(1, :)';
%   P2T = P(2, :)';
%   P3T = P(3, :)';
%
%   P1T_dash = P_dash(1, :)';
%   P2T_dash = P_dash(2, :)';
%   P3T_dash = P_dash(3, :)';
%
%   A = [x * P3T - P1T;
%        y * P3T - P2T;
%        x * P3T_dash - P1T_dash;
%        y * P3T_dash - P2T_dash];
%
%   svd(A);
% endfunction
