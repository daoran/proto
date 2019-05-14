addpath(genpath("prototype"));

camera = camera_init([680; 480], 90.0);
calib_target = calib_target_init(nb_rows=4, nb_cols=4, tag_size=0.2);
data = trajectory_simulate(camera, calib_target);

assert(length(data.time) > 0);
assert(isfield(data, "camera"));
assert(isfield(data, "target"));

assert(isfield(data, "q_WT"));
assert(isfield(data, "r_WT"));
assert(length(data.q_WT) > 0);
assert(length(data.r_WT) > 0);

assert(isfield(data, "q_WC"));
assert(isfield(data, "r_WC"));
assert(length(data.q_WC) > 0);
assert(length(data.r_WC) > 0);

assert(length(data.z_data) == length(data.time));
assert(length(data.p_data) == length(data.time));
