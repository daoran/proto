addpath(genpath("prototype"));

p = euler2quat(deg2rad([1.0, 2.0, 3.0]));
q = euler2quat(deg2rad([3.0, 2.0, 1.0]));
r = quat_mul(p, q);
