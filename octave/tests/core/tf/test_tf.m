addpath(genpath("proto"));

pose = [1.0; 0.0; 0.0; 0.0; 1.0; 2.0; 3.0];
T = tf(pose);
assert(isequal(T(1:3, 1:3), eye(3)) == 1);
assert(isequal(T(1:3, 4), [1; 2; 3]) == 1);

C = [1.0, 0.0, 0.0;
     0.0, 2.0, 0.0;
     0.0, 0.0, 3.0];
r = [1.0; 2.0; 3.0];
T = tf(C, r);
assert(isequal(T(1:3, 1:3), C) == 1);
assert(isequal(T(1:3, 4), r) == 1);
