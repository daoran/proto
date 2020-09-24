addpath(genpath("proto"));

data = [1; 2; 3];
param = landmark_init(0, data);

assert(param.type == "landmark");
assert(param.lm_id == 0);
assert(param.param == [1; 2; 3]);
assert(param.min_dims == 3);
