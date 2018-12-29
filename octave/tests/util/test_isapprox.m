addpath(genpath("prototype"));

% Test vector
A = [1.0; 2.0; 3.0];
B = [1.0; 3.0; 3.0];
assert(isapprox(A, B) == 0);
assert(isapprox(A, A) == 1);

% Test matrix
A = rand(4, 4);
B = rand(4, 4);
assert(isapprox(A, B) == 0);
assert(isapprox(A, A) == 1);
