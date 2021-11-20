addpath(genpath("proto"));

% Create camera
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Calibration target
calib_target = calib_target_init(nb_rows=4, nb_cols=4, tag_size=0.2);

% Simulate trajectory
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
