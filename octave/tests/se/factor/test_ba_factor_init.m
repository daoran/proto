addpath(genpath("proto"));

% Setup camera pose T_WC
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC = tf(rot, trans);
data = [rot; trans];
cam_pose = pose_init(0, data);

% Setup cam0
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
cam_params = camera_init(cam_idx, resolution,
                         proj_model, dist_model,
                         proj_params, dist_params);

% Setup landmark
landmark = landmark_init(0, [10; rand(2, 1)]);
p_C = tf_point(inv(T_WC), landmark.param);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);

% Setup graph
graph = graph_init();
[graph, cam_pose_id] = graph_add_param(graph, cam_pose);
[graph, landmark_id] = graph_add_param(graph, landmark);
[graph, cam_params_id] = graph_add_param(graph, cam_params);

% Create BA factor
ts = 0;
param_ids = [cam_pose_id; landmark_id; cam_params_id];
ba_factor = ba_factor_init(ts, param_ids, z);
assert(isfield(ba_factor, "ts"));
assert(isfield(ba_factor, "param_ids"));
assert(isfield(ba_factor, "z"));
assert(isfield(ba_factor, "covar"));
