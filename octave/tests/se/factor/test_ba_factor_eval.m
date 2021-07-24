addpath(genpath("proto"));

% Setup camera pose T_WC
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC = tf(rot, trans);
data = tf_param(T_WC);
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
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Setup landmark
landmark = landmark_init(0, [10; rand(2, 1)]);
p_C = tf_point(inv(T_WC), landmark.param);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);

% Setup graph
graph = graph_init();
[graph, cam_pose_id] = graph_add_param(graph, cam_pose);
[graph, landmark_id] = graph_add_param(graph, landmark);
[graph, cam_params_id] = graph_add_param(graph, camera);

% Create factor
ts = 0;
param_ids = [cam_pose_id; landmark_id; cam_params_id];
ba_factor = ba_factor_init(ts, param_ids, z);

% Evaluate factor
params = graph_get_params(graph, ba_factor.param_ids);
[r, jacobians] = ba_factor_eval(ba_factor, params);

% Test jacobians
step_size = 1e-8;
threshold = 1e-4;
check_factor_jacobian(ba_factor, params, 1, "J_cam_pose", step_size, threshold);
check_factor_jacobian(ba_factor, params, 2, "J_landmark", step_size, threshold);
check_factor_jacobian(ba_factor, params, 3, "J_cam_params", step_size, threshold);
