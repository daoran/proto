addpath(genpath("proto"));

% Setup sensor pose T_WS
rot = euler2quat(deg2rad([0.1; 0.1; 0.1]));
trans = [0.01; 0.02; 0.03];
T_WS = tf(rot, trans);
data = tf_param(T_WS);
sensor_pose = pose_init(0, data);

% Setup imu-camera extrinsics
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.1; 0.2; 0.3];
T_SC = tf(rot, trans);
data = tf_param(T_SC);
imucam_exts = pose_init(0, data);

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
p_C = tf_point(inv(T_SC) * inv(T_WS), landmark.param);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);

% Setup graph
graph = graph_init();
[graph, sensor_pose_id] = graph_add_param(graph, sensor_pose);
[graph, imucam_exts_id] = graph_add_param(graph, imucam_exts);
[graph, landmark_id] = graph_add_param(graph, landmark);
[graph, cam_params_id] = graph_add_param(graph, camera);

% Create factor
ts = 0;
param_ids = [sensor_pose_id; imucam_exts_id; landmark_id; cam_params_id];
cam_factor = cam_factor_init(ts, param_ids, z);

% Evaluate factor
params = graph_get_params(graph, cam_factor.param_ids);
[r, jacobians] = cam_factor_eval(cam_factor, params);

% Test jacobians
step_size = 1e-8;
threshold = 1e-4;
check_factor_jacobian(cam_factor, params, 1, "J_sensor_pose", step_size, threshold);
check_factor_jacobian(cam_factor, params, 2, "J_imucam_exts", step_size, threshold);
check_factor_jacobian(cam_factor, params, 3, "J_landmark", step_size, threshold);
check_factor_jacobian(cam_factor, params, 4, "J_cam_params", step_size, threshold);
