addpath(genpath("prototype"));

x = [1.0; 0.0; 0.0];
R = rotz(deg2rad(90.0));
x_prime = R * x;
assert(norm(x_prime - [0.0; 1.0; 0.0]) < 1e-5);
