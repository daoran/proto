addpath(genpath("prototype"));

imu = imu_create();
assert(isfield(imu, "q_IG"));
assert(isfield(imu, "b_g"));
assert(isfield(imu, "v_IG"));
assert(isfield(imu, "b_a"));
assert(isfield(imu, "p_IG"));
