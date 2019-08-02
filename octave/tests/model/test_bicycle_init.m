addpath(genpath("proto"));

x = 0.1;
y = 0.2;
theta = 0.3;
l = 0.5;
bicycle = bicycle_init(x, y, theta, l);

assert(rows(bicycle.state) == 3);
assert(columns(bicycle.state) == 1);
assert(bicycle.state(1) == 0.1);
assert(bicycle.state(2) == 0.2);
assert(bicycle.state(3) == 0.3);
assert(bicycle.l == 0.5);
