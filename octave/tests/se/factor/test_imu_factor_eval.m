addpath(genpath("proto"));

% Simulate imu data
save_path = "/tmp/test_imu_factor_eval.data";
if length(glob(save_path)) == 0
  sim_data = sim_imu(0.5, 1.0);
  save("-binary", save_path, "sim_data");
else
  load(save_path);
endif

% Test settings
window_size = 10;
g = [0.0; 0.0; 9.81];

% IMU params
imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

for i = 1:window_size:(length(sim_data.imu_time)-window_size);
  % i = 361
  start_idx = i;
  end_idx = start_idx + window_size - 1;

  % IMU buffer
  imu_buf = {};
  imu_buf.ts = sim_data.imu_time(start_idx:end_idx);
  imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
  imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

  % Pose i
  T_WS_i = sim_data.imu_poses{start_idx};
  pose_i = pose_init(imu_buf.ts(1), T_WS_i);

  % Speed and bias i
  vel_i = sim_data.imu_vel(:, start_idx);
  % ba_i = zeros(3, 1);
  % bg_i = zeros(3, 1);
  ba_i = 1e-3 * ones(3, 1);
  bg_i = 1e-4 * ones(3, 1);
  sb_i = sb_init(imu_buf.ts(1), vel_i, bg_i, ba_i);

  % Pose j
  T_WS_j = sim_data.imu_poses{end_idx};
  pose_j = pose_init(imu_buf.ts(end), T_WS_j);

  % Speed and bias j
  vel_j = sim_data.imu_vel(:, end_idx);
  % ba_j = zeros(3, 1);
  % bg_j = zeros(3, 1);
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
  imu_factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i, "midpoint");

  % Evaluate factor
  params = graph_get_params(graph, imu_factor.param_ids);
  imu_factor.sqrt_info = eye(15);
  [r, jacobians] = imu_factor_eval(imu_factor, params);

  % Check propagation
  T_WS_j_est = T_WS_i * tf(imu_factor.dC, imu_factor.dr);
  C_WS_j_est = tf_rot(T_WS_j_est);
  C_WS_j_gnd = tf_rot(T_WS_j);
  % -- Position
  trans_diff = norm(tf_trans(T_WS_j) - tf_trans(T_WS_j_est));
  assert(trans_diff < 0.05);
  % -- Rotation
  dC = C_WS_j_gnd' * C_WS_j_est;
  dq = quat_normalize(rot2quat(dC));
  dC = quat2rot(dq);
  rpy_diff = rad2deg(acos((trace(dC) - 1.0) / 2.0));
  assert(rpy_diff < 1.0);

  % Test jacobians
  step_size = 1e-8;
  threshold = 1e-3;

  % [_, jacobians] = imu_factor_eval(imu_factor, params);
  % octave_Q = imu_factor.Q;
  % octave_F = imu_factor.state_F;
  % octave_P = imu_factor.state_P;
  % octave_info = inv(octave_P);
  % octave_sqrt_info = imu_factor.sqrt_info;
  % octave_J_pose_i = jacobians{1};
  % octave_J_sb_i = jacobians{2};
  % octave_J_pose_j = jacobians{3};
  % octave_J_sb_j = jacobians{4};
  %
  % cpp_Q = csvread("/tmp/Q.csv");
  % cpp_F = csvread("/tmp/F.csv");
  % cpp_P = csvread("/tmp/P.csv");
  % cpp_info = csvread("/tmp/info.csv");
  % cpp_sqrt_info = csvread("/tmp/sqrt_info.csv");
  % cpp_J_pose_i = csvread("/tmp/J_pose_i.csv")(1:15, 1:6);
  % cpp_J_sb_i = csvread("/tmp/J_sb_i.csv");
  % cpp_J_pose_j = csvread("/tmp/J_pose_j.csv")(1:15, 1:6);
  % cpp_J_sb_j = csvread("/tmp/J_sb_j.csv");

  % A = octave_Q;
  % B = cpp_Q;

  % A = octave_F;
  % B = cpp_F;

  % A = octave_P;
  % B = cpp_P;

  % compare_matrices("Octave", A, "C++", B);
  % ginput()

  check_factor_jacobian(imu_factor, params, 1, "J_pose_i", step_size, threshold);
  check_factor_jacobian(imu_factor, params, 2, "J_sb_i", step_size, threshold);
  check_factor_jacobian(imu_factor, params, 3, "J_pose_j", step_size, threshold);
  check_factor_jacobian(imu_factor, params, 4, "J_sb_j", step_size, threshold);
  printf("\n");
  % break;
endfor
