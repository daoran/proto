addpath(genpath("proto"));

sim_data = sim_imu(0.5, 1.0);
window_size = 10
g = [0.0; 0.0; 9.81];

% IMU params
imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

for i = 1:window_size:(length(sim_data.imu_time)-window_size);
  start_idx = i;
  end_idx = start_idx + window_size - 1;

  % IMU buffer
  imu_buf = {};
  imu_buf.ts = sim_data.imu_time(start_idx:end_idx);
  imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
  imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

  % Pose i
  pose_i = pose_init(imu_buf.ts(1), sim_data.imu_poses{start_idx});

  % Speed and bias i
  vel_i = sim_data.imu_vel(:, start_idx);
  ba_i = 1e-4 * ones(3, 1);
  bg_i = 1e-5 * ones(3, 1);
  sb_i = sb_init(imu_buf.ts(1), vel_i, bg_i, ba_i);

  % Pose j
  pose_j = pose_init(imu_buf.ts(end), sim_data.imu_poses{end_idx});

  % Speed and bias j
  vel_j = sim_data.imu_vel(:, end_idx);
  ba_j = 1e-4 * ones(3, 1);
  bg_j = 1e-5 * ones(3, 1);
  sb_j = sb_init(imu_buf.ts(end), vel_j, bg_j, ba_j);

  % Setup graph
  graph = graph_init();
  [graph, pose_i_id] = graph_add_param(graph, pose_i);
  [graph, sb_i_id] = graph_add_param(graph, sb_i);
  [graph, pose_j_id] = graph_add_param(graph, pose_j);
  [graph, sb_j_id] = graph_add_param(graph, sb_j);

  % Create factor
  param_ids = [pose_i_id; sb_i_id; pose_j_id; sb_j_id];
  imu_factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i);

  % Evaluate factor
  params = graph_get_params(graph, imu_factor.param_ids);
  v_i_est = params{2}.param(1:3);
  ba_i_est = params{2}.param(4:6);
  bg_i_est = params{2}.param(7:9) + 1e-4 * ones(3, 1);
  params{2}.param = [v_i_est; ba_i_est; bg_i_est];
  [r, jacobians] = imu_factor_eval(imu_factor, params);

  % Test jacobians
  step_size = 1e-8;
  threshold = 1e-4;
  imu_factor.sqrt_info = eye(15); % Numerical diff will not work if we don't hack this
  check_factor_jacobian(imu_factor, params, 1, "J_pose_i", step_size, threshold)
  check_factor_jacobian(imu_factor, params, 2, "J_sb_i", step_size, threshold)
  check_factor_jacobian(imu_factor, params, 3, "J_pose_j", step_size, threshold)
  check_factor_jacobian(imu_factor, params, 4, "J_sb_j", step_size, threshold)
  printf("\n");
endfor
