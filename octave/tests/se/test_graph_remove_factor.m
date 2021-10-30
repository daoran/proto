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

% Setup feature
feature = feature_init(0, [10; rand(2, 1)]);
p_C = tf_point(inv(T_WC), feature.param);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);

% Create graph
graph = graph_init();

[graph, cam_pose_id] = graph_add_param(graph, cam_pose);
[graph, feature_id] = graph_add_param(graph, feature);
[graph, cam_params_id] = graph_add_param(graph, cam_params);

% Add factor
param_ids = [cam_pose_id, feature_id, cam_params_id];
ba_factor = ba_factor_init(0, param_ids, z);
[graph, factor_id] = graph_add_factor(graph, ba_factor);
assert(length(graph.factors) == 1);

% Remove factor
graph = graph_remove_factor(graph, factor_id);
length(graph.factors)
