addpath(genpath("proto"));

% Setup pose T_WB
rot = euler2quat(deg2rad([0.0; 0.0; 0.0]));
trans = [0.0; 0.0; 0.0];
T_WB = tf(rot, trans);
data = tf_param(T_WB);
pose = pose_init(0, data);

% Setup camera extrinsics
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.01; 0.02; 0.03];
% trans = [0.0; 0.0; 0.0];
T_BC0 = tf(rot, trans);
data = tf_param(T_BC0);
exts = pose_init(0, data);

% Setup cam0
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 90.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx; fy; cx; cy];
dist_params = [0.0; 0.0; 0.0; 0.0];
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Setup feature
% feature = feature_init(0, [10; rand(2, 1)]);
feature = feature_init(0, [1.0; 0.0; 0.0]);
p_C0 = tf_point(inv(T_BC0) * inv(T_WB), feature.param);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C0);

% Setup graph
graph = graph_init();
[graph, pose_id] = graph_add_param(graph, pose);
[graph, exts_id] = graph_add_param(graph, exts);
[graph, feature_id] = graph_add_param(graph, feature);
[graph, cam_params_id] = graph_add_param(graph, camera);

% Create factor
ts = 0;
param_ids = [pose_id; exts_id; feature_id; cam_params_id];
cam_factor = cam_factor_init(ts, param_ids, z);

% Evaluate factor
params = graph_get_params(graph, cam_factor.param_ids);
[r, jacobians] = cam_factor_eval(cam_factor, params);

% jacobians{1}
% % jacobians{2}
% % jacobians{3}
% % jacobians{4}
%
% % Test jacobians
% step_size = 1e-8;
% threshold = 1e-4;
% check_factor_jacobian(cam_factor, params, 1, "J_pose", step_size, threshold);
% check_factor_jacobian(cam_factor, params, 2, "J_exts", step_size, threshold);
% check_factor_jacobian(cam_factor, params, 3, "J_feature", step_size, threshold);
% check_factor_jacobian(cam_factor, params, 4, "J_cam_params", step_size, threshold);
