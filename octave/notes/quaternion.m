addpath(genpath("proto"));
pkg load symbolic;

% syms qx qy qz qw;
% epsilon = [qx; qy; qz];
% eta = qw;
% q = [eta * eye(3) - skew(epsilon), epsilon;
%      -epsilon', eta];

% q = [eta * eye(3) - skew(epsilon), epsilon;
%      -epsilon', eta];
% q

% + pz qw - px qy + py qx + pw qz 
% - px qx - py qy - pz qz + pw qw 
% + qx pw + py qz - pz qy + pw qx 
% - py qw + pz qx + px qz + pw qy 

% pw, -px, -py, -pz,
% px,  pw, -pz,  py,
% py,  pz,  pw, -px,
% pz, -py,  px,  pw


p_ = euler2quat([deg2rad(0.4), deg2rad(0.5), deg2rad(0.6)]);
q_ = euler2quat([deg2rad(0.1), deg2rad(0.2), deg2rad(0.3)]);

p = [p_(2); p_(3); p_(4); p_(1)];
q = [q_(2); q_(3); q_(4); q_(1)];

px = p(1);
py = p(2);
pz = p(3);
pw = p(4);

P = [pw, -pz,  py, px;
     pz,  pw, -px, py;
     -py,  px,  pw, pz;
     -px, -py, -pz, pw];
P * q

    pw, -px, -py, -pz;
    px, pw, -pz, py;
    py, pz, pw, -px;
    pz, -py, px, pw;

% Q = [ qw,  qz, -qy, qx;
%      -qz,  qw,  qx, qy;
%       qy, -qx,  qw, qz;
%      -qx, -qy, -qz, qw];

quat_lmul(p_, q_)

% px = p_(1);
% py = p_(2);
% pz = p_(3);
% pw = p_(4);
%
% lprod = [
%   pw, -px, -py, -pz;
%   px, pw, -pz, py;
%   py, pz, pw, -px;
%   pz, -py, px, pw;
% ];
% lprod * q_



% Test Jacobian
% q = euler2quat([deg2rad(0.1), deg2rad(0.2), deg2rad(0.3)]);
% p = [0.1; 0.2; 0.3];
% qw = q(1);
% qv = q(2:4);

% % Check jacobian of C(q) * a w.r.t. q
% dp__dqw = 2 * (qw * p + cross(qv, p))
% dp__dqv = 2 * (qv' * p * eye(3) + qv * p' - p * qv' - qw * skew(p))
% dp__dq = [dp__dqw, dp__dqv];
%
% fdiff = zeros(3, 4);
% step_size = 1.0e-8;
% C = quat2rot(q);
% x = C * p;
%
% C_diff = quat2rot(q + [step_size; 0; 0; 0]);
% fdiff(1:3, 1) = ((C_diff * p) - x) / step_size;
%
% C_diff = quat2rot(q + [0; step_size; 0; 0]);
% fdiff(1:3, 2) = ((C_diff * p) - x) / step_size;
%
% C_diff = quat2rot(q + [0; 0; step_size; 0]);
% fdiff(1:3, 3) = ((C_diff * p) - x) / step_size;
%
% C_diff = quat2rot(q + [0; 0; 0; step_size]);
% fdiff(1:3, 4) = ((C_diff * p) - x) / step_size;
%
% fdiff
% dp__dq
