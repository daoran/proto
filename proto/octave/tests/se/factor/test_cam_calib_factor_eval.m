addpath(genpath("proto"));

% Fiducial pose T_WF
rot = euler2quat(deg2rad([90.1; 0; -90.2]));
trans = [0.1; 0.2; 0.0];
T_WF = tf(rot, trans);

% Object point
r_FFi = [0.01; 0.02; 0.0];

% Relative camera pose T_C0F
rot = euler2quat(deg2rad([-180.1; 0.01; 0.2]));
trans = [-0.1; 0.2; 10.0];
T_C0F = tf(rot, trans);
rel_pose = pose_init(0, tf_param(T_C0F));

% Camera extrinsics T_C0Ci
rot = euler2quat(deg2rad([0.0; 0.0; 0.0]));
trans = [0.1; 0.2; 0.01];
T_C0Ci = tf(rot, trans);
extrinsics = extrinsics_init(0, tf_param(T_C0Ci));

% figure();
% hold on;
% draw_frame(T_WF);
% draw_frame(T_WF * inv(T_C0F));
% view(3);
% ginput();

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
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Setup graph
graph = graph_init();
[graph, rel_pose_id] = graph_add_param(graph, rel_pose);
[graph, extrinsics_id] = graph_add_param(graph, extrinsics);
[graph, cam_params_id] = graph_add_param(graph, camera);

% Create factor
ts = 0;
param_ids = [rel_pose_id; extrinsics_id; cam_params_id];
r_CiFi = tf_point(inv(T_C0Ci) * T_C0F, r_FFi);
z = pinhole_radtan4_project(proj_params, dist_params, r_CiFi);
cam_calib_factor = cam_calib_factor_init(ts, param_ids, z, r_FFi);

% Evaluate factor
params = graph_get_params(graph, cam_calib_factor.param_ids);
[r, jacobians] = cam_calib_factor_eval(cam_calib_factor, params);

% Test jacobians
step_size = 1e-8;
threshold = 1e-4;
check_factor_jacobian(cam_calib_factor, params, 1, "J_rel_pose", step_size, threshold);
check_factor_jacobian(cam_calib_factor, params, 2, "J_extrinsics", step_size, threshold);
check_factor_jacobian(cam_calib_factor, params, 3, "J_cam_params", step_size, threshold);
