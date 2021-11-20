addpath(genpath("proto"));

C = euler321([0.0; 0.0; 0.0]);
assert(isequal(C, eye(3)) == 1);
