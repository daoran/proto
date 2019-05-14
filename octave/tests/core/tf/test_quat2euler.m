addpath(genpath("prototype"));

rpy = deg2rad([1.0; 2.0; 3.0]);
q = euler2quat(rpy);
euler = quat2euler(q);
assert(rows(q) == 4);
assert(columns(q) == 1);
assert(abs(euler(1) - rpy(1)) < 1e-5);
assert(abs(euler(2) - rpy(2)) < 1e-5);
assert(abs(euler(3) - rpy(3)) < 1e-5);
