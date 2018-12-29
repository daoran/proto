addpath(genpath("prototype"));

T = eye(4);
T_diff = perturb_rot(T, 1e-5, 1);
assert(isequal(T, T_diff) == 0);

T_diff = perturb_rot(T, 1e-5, 2);
assert(isequal(T, T_diff) == 0);

T_diff = perturb_rot(T, 1e-5, 3);
assert(isequal(T, T_diff) == 0);
