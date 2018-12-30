addpath(genpath("prototype"));

% Test vector
p = [1.0; 2.0; 3.0];
hp = homogeneous(p);
assert(length(hp) == 4);

% Test matrix
p = rand(3, 100);
hp = homogeneous(p);
assert(rows(hp) == 4);
assert(columns(hp) == 100);
assert(isequal(hp(4, :), ones(1, 100)));
