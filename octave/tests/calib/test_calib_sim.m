addpath(genpath("proto"));

% Setup calibration target
calib_target = calib_target_init();
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [1.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);

% Setup camera
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_model = "pinhole";
dist_model = "radtan4";
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera = camera_init(cam_idx, resolution,
                     proj_model, dist_model,
                     proj_params, dist_params);


% Simulation calibration data
% debug = true;
debug = false;
nb_poses = 20;
data = calib_sim(calib_target, T_WT, camera, nb_poses, debug);

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
assert(length(data.point_ids_data) == length(data.time));
