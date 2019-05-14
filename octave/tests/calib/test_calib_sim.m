addpath(genpath("prototype"));

% Setup calibration target
calib_target = calib_target_init();
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [1.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);

% Setup camera
fov = 90.0;
image_size = [640; 480];
camera = camera_init(image_size, fov);

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
