addpath(genpath("prototype"));

C = [1.0, 0.0, 0.0;
     0.0, 1.0, 0.0;
     0.0, 0.0, 1.0];
r = [1.0; 2.0; 3.0];
assert(isequal(C, tf_rot(T)) == 1);
