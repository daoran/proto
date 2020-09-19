addpath(genpath("proto"));

data = [1; 2; 3];
param = landmark_t(0, data);

assert(param.type_info == "landmark_t");
assert(param.ts == 0);
assert(param.data == [1; 2; 3]);
assert(param.min_size == 3);

param.print()
