addpath(genpath("prototype"));

q = [1.0; 0.0; 0.0; 0.0];
n = quat_norm(q);
assert(n == 1.0);
