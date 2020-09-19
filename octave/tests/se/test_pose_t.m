addpath(genpath("proto"));

rot = [1.0; 0.0; 0.0; 0.0];
trans = [0.0; 0.0; 0.0];
data = [rot; trans];
param = pose_t(0, data);

assert(param.type_info == "pose_t");
assert(param.ts == 0);
assert(param.data == data);
assert(param.min_size == 6);

param.print()
