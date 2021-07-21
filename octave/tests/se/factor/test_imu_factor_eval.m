addpath(genpath("proto"));

sim_data = sim_imu(0.5, 1.0);
window_size = 10
g = [0.0; 0.0; 9.81];

start_idx = 10;
end_idx = start_idx + window_size - 1;

% IMU buffer
imu_buf = {};
imu_buf.ts = sim_data.time(start_idx:end_idx);
imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

% IMU params
imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

% Pose i
pose_i = pose_init(imu_buf.ts(start_idx), sim_data.poses{start_idx});

% Speed and bias i
vel_i = sim_data.vel(:, start_idx);
ba_i = zeros(3, 1);
bg_i = zeros(3, 1);
sb_i = sb_init(imu_buf.ts(1), vel_i, bg_i, ba_i);

% Pose j
pose_j = pose_init(imu_buf.ts(end), sim_data.poses{end_idx});

% Speed and bias j
vel_j = sim_data.vel(:, end_idx);
ba_j = zeros(3, 1);
bg_j = zeros(3, 1);
sb_j = sb_init(imu_buf.ts(end), vel_j, bg_j, ba_j);

% Setup graph
graph = graph_init();
[graph, pose_i_id] = graph_add_param(graph, pose_i);
[graph, sb_i_id] = graph_add_param(graph, sb_i);
[graph, pose_j_id] = graph_add_param(graph, pose_j);
[graph, sb_j_id] = graph_add_param(graph, sb_j);

% Create imu factor
param_ids = [pose_i_id; sb_i_id; pose_j_id; sb_j_id];
imu_factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i);

% Evaluate imu factor
params = graph_get_params(graph, imu_factor.param_ids);
[r, jacobians] = imu_factor_eval(imu_factor, params);

step_size = 1e-8;
threshold = 1e-4;
check_factor_jacobians(@imu_factor_eval, imu_factor, params, 1, "J_pose_i", step_size, threshold)
% check_factor_jacobians(@imu_factor_eval, imu_factor, params, 2, "J_sb_i", step_size, threshold)
% check_factor_jacobians(@imu_factor_eval, imu_factor, params, 3, "J_pose_j", step_size, threshold)
% check_factor_jacobians(@imu_factor_eval, imu_factor, params, 4, "J_sb_j", step_size, threshold)
