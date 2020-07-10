addpath(genpath("proto"));

C = [1.0, 0.0, 0.0;
     0.0, 2.0, 0.0;
     0.0, 0.0, 3.0];
r = [1.0; 2.0; 3.0];
T = tf(C, r);
assert(isequal(T(1:3, 1:3), C) == 1);
assert(isequal(T(1:3, 4), r) == 1);
