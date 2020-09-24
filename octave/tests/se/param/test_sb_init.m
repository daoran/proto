addpath(genpath("proto"));

v = [0.1; 0.2; 0.3];
bg = [1.1; 1.2; 1.3];
ba = [2.1; 2.2; 3.3];
param = [v; bg; ba];
sb = sb_init(0, v, bg, ba);

assert(sb.type == "sb");
assert(sb.ts == 0);
assert(sb.param == param);
assert(sb.min_dims == 9);
