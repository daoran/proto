addpath(genpath("prototype"));

C = euler321([0.0; 0.0; 0.0]);
assert(isequal(C, eye(3)) == 1);
