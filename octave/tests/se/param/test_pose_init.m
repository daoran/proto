addpath(genpath("proto"));

rot = [1.0; 0.0; 0.0; 0.0];
trans = [0.0; 0.0; 0.0];
param = [rot; trans];
pose = pose_init(0, param);

assert(pose.type == "pose");
assert(pose.ts == 0);
assert(pose.param == param);
assert(pose.min_dims == 6);
