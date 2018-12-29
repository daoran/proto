addpath(genpath("prototype"));

M = skew([1.0; 2.0; 3.0]);
expected = [0.0, -3.0, 2.0;
            3.0, 0.0, -1.0;
            -2.0, 1.0, 0.0];
assert(isequal(M, expected) == 1);
