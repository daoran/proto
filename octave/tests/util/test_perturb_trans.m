addpath(genpath("prototype"));

T = eye(4);
T_diff = perturb_trans(T, 1e-5, 1);
assert(T_diff(1, 4) > 0);
assert(T_diff(2, 4) == 0);
assert(T_diff(3, 4) == 0);
assert(isequal(T, T_diff) == 0);

T_diff = perturb_trans(T, 1e-5, 2);
assert(T_diff(1, 4) == 0);
assert(T_diff(2, 4) > 0);
assert(T_diff(3, 4) == 0);
assert(isequal(T, T_diff) == 0);

T_diff = perturb_trans(T, 1e-5, 3);
assert(T_diff(1, 4) == 0);
assert(T_diff(2, 4) == 0);
assert(T_diff(3, 4) > 0);
assert(isequal(T, T_diff) == 0);
