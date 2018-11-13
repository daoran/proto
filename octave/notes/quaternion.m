addpath(genpath("prototype"));
pkg load symbolic;

syms qx qy qz qw;

epsilon = [qx; qy; qz];
eta = qw;

q = [eta * eye(3) - skew(epsilon), epsilon;
     -epsilon', eta];
q
