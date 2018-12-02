addpath(genpath("prototype"));
% pkg load symbolic;
%
% syms qx qy qz qw;
% epsilon = [qx; qy; qz];
% eta = qw;
% q = [eta * eye(3) - skew(epsilon), epsilon;
%      -epsilon', eta];
% q

q = euler2quat([deg2rad(0.1), deg2rad(0.2), deg2rad(0.3)]);
p = [0.1; 0.2; 0.3];
qw = q(1);
qv = q(2:4);


% Check jacobian of C(q) * a w.r.t. q
dp__dqw = 2 * (qw * p + cross(qv, p))
dp__dqv = 2 * (qv' * p * eye(3) + qv * p' - p * qv' - qw * skew(p))
dp__dq = [dp__dqw, dp__dqv];

fdiff = zeros(3, 4);
step_size = 1.0e-8;
C = quat2rot(q);
x = C * p;

C_diff = quat2rot(q + [step_size; 0; 0; 0]);
fdiff(1:3, 1) = ((C_diff * p) - x) / step_size;

C_diff = quat2rot(q + [0; step_size; 0; 0]);
fdiff(1:3, 2) = ((C_diff * p) - x) / step_size;

C_diff = quat2rot(q + [0; 0; step_size; 0]);
fdiff(1:3, 3) = ((C_diff * p) - x) / step_size;

C_diff = quat2rot(q + [0; 0; 0; step_size]);
fdiff(1:3, 4) = ((C_diff * p) - x) / step_size;

fdiff
dp__dq
