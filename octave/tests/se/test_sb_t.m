addpath(genpath("proto"));

v = [0.1; 0.2; 0.3];
bg = [1.1; 1.2; 1.3];
ba = [2.1; 2.2; 3.3];
data = [v; bg; ba];
param = sb_t(0, v, bg, ba);

assert(param.type_info == "sb_t");
assert(param.ts == 0);
assert(param.data == data);
assert(param.min_size == 9);

param.print()
