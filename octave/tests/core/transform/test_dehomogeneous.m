addpath(genpath("prototype"));

hp = [1.0; 2.0; 3.0; 1.0];
p = dehomogeneous(hp);
assert(p(1) == 1.0);
assert(p(2) == 2.0);
assert(p(3) == 3.0);
assert(rows(p) == 3);
assert(columns(p) == 1);
