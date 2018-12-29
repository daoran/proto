addpath(genpath("prototype"));

n = normalize([1.0, 2.0, 3.0]);
assert(norm(n) == 1);
