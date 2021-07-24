addpath(genpath("proto"));

% Setup camera pose T_WC
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.1; 0.2; 0.3];
T_WC = tf(rot, trans);
data = tf_param(T_WC);
pose_meas = pose_init(0, data);
pose_est = pose_update(pose_meas, [0.1; 0.2; 0.3; 0.1; 0.2; 0.3]);

% Setup graph
graph = graph_init();
[graph, cam_pose_id] = graph_add_param(graph, pose_est);

% Create factor
ts = 0;
param_ids = [cam_pose_id];
pose_factor = pose_factor_init(ts, param_ids, tf(pose_meas.param));

% Evaluate factor
params = graph_get_params(graph, pose_factor.param_ids);
[r, jacobians] = pose_factor_eval(pose_factor, params);

% Test jacobians
step_size = 1e-8;
threshold = 1e-4;
check_factor_jacobian(pose_factor, params, 1, "J_pose", step_size, threshold);
