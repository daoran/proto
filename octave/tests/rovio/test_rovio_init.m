addpath(genpath("prototype"));

rovio = rovio_init();

assert(isfield(rovio.imu, "r"));
assert(isfield(rovio.imu, "v"));
assert(isfield(rovio.imu, "q"));
assert(isfield(rovio.imu, "b_f"));
assert(isfield(rovio.imu, "b_w"));

assert(isfield(rovio.imu_cam, "p"));
assert(isfield(rovio.imu_cam, "q"));
