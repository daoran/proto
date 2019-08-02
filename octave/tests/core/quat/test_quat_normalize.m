addpath(genpath("proto"));

q = [1.0; 0.1; 0.2; 0.3];
q = quat_normalize(q);
assert(norm(q) == 1.0);
