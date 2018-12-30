addpath(genpath("prototype"));

C = [1.0, 0.0, 0.0;
     0.0, 2.0, 0.0;
     0.0, 0.0, 3.0];
r = [1.0; 2.0; 3.0];
T = tf(C, r);
[C_retval, r_retval] = tf_decompose(T);
assert(isequal(C_retval, C) == 1);
assert(isequal(r_retval, r) == 1);
