addpath(genpath("proto"));

function visualize(fig_title, graph, poses_gnd, vels_gnd)
  % Extract pose data
  pos_gnd = [];
  pos_est = [];
  rpy_gnd = [];
  rpy_est = [];

  for k = 1:length(graph.factors)
    T_WS = poses_gnd{k};
    pos_gnd = [pos_gnd, tf_trans(T_WS)];
    rpy_gnd = [rpy_gnd, rad2deg(rot2euler(tf_rot(T_WS)))];

    factor = graph.factors{k};
    params = graph_get_params(graph, factor.param_ids);
    pose_i = params{1};
    sb_i = params{2};
    T_WS = tf(pose_i.param);
    pos_est = [pos_est, tf_trans(T_WS)];
    rpy_est = [rpy_est, rad2deg(rot2euler(tf_rot(T_WS)))];
  endfor

  % Extract velocity data
  t = [];
  v_gnd = [];
  v_est = [];
  for k = 1:length(graph.factors)
    factor = graph.factors{k};
    params = graph_get_params(graph, factor.param_ids);
    ts = factor.imu_buf.ts(1);
    sb_i = params{2};
    v = vels_gnd{k};

    t = [t; ts];
    v_gnd = [v_gnd, vels_gnd{k}];
    v_est = [v_est, sb_i.param(1:3)];
  endfor

  % Plot
  figure();
  title(fig_title);

  % -- X-Y Position
  subplot(3, 3, [1, 4, 7]);
  hold on;
  plot(pos_gnd(1, :), pos_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(pos_est(1, :), pos_est(2, :), 'rx', 'linewidth', 2.0);
  axis('equal');
  xlabel("Displacement in x-direction [m]");
  ylabel("Displacement in y-direction [m]");
  lgnd = legend("Ground Truth", "Estimate");
  legend boxoff;
  set(lgnd, 'color', 'none');
  title("X-Y Position");

  % -- Velocities
  subplot(3, 3, 2);
  hold on;
  plot(t, v_gnd(1, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(1, :), 'r-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  title("Velocity");

  subplot(3, 3, 5);
  hold on;
  plot(t, v_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(2, :), 'g-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");

  subplot(3, 3, 8);
  hold on;
  plot(t, v_gnd(3, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");

  % -- Attitude
  subplot(3, 3, 3);
  hold on;
  plot(t, rpy_gnd(1, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(1, :), 'r-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  title("Attitude");

  subplot(3, 3, 6);
  hold on;
  plot(t, rpy_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(2, :), 'g-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");

  subplot(3, 3, 9);
  hold on;
  plot(t, rpy_gnd(3, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");

  pause(0.1);
  refresh();
endfunction

% Simulate imu data
sim_data = sim_imu(0.5, 1.0);
window_size = 10;
g = [0.0; 0.0; 9.81];

% IMU params
imu_params = {};
% imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
% imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
% imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
% imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.
imu_params.noise_acc = 1e-8;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 1e-8;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 1e-8;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 1e-8;   % gyroscope bias random work noise stddev.

% Create graph
graph = graph_init();

% Add initial pose and speed and biases
% -- Create Pose i
pose_i = pose_init(sim_data.time(1), sim_data.poses{1});
% -- Create Speed and bias i
vel_i = sim_data.vel(:, 1);
ba_i = zeros(3, 1);
bg_i = zeros(3, 1);
sb_i = sb_init(sim_data.time(1), vel_i, bg_i, ba_i);
% -- Add pose_i and sb_i to graph
[graph, pose_i_id] = graph_add_param(graph, pose_i);
[graph, sb_i_id] = graph_add_param(graph, sb_i);
% -- Keep track of ground truth poses
poses_gnd = {};
pose_idx = 1;
poses_gnd{pose_idx} = sim_data.poses{1};
pose_idx += 1;
% -- Keep track of ground truth velocities
vels_gnd = {};
vel_idx = 1;
vels_gnd{vel_idx} = sim_data.vel(:, 1);
vel_idx += 1;

% Add imu factors
for start_idx = 1:window_size:(length(sim_data.time)-window_size);
  end_idx = start_idx + window_size - 1;

  % IMU buffer
  imu_buf = {};
  imu_buf.ts = sim_data.time(start_idx:end_idx);
  imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
  imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

  % Pose j
  T_WS_j = sim_data.poses{end_idx};
  T_WS_j(1:3, 1:3) = T_WS_j(1:3, 1:3) * Exp(normrnd(0.0, 0.005, 3, 1)); % Add noise to rotation
  T_WS_j(1:3, 4) += normrnd(0.0, 0.5, 3, 1); % Add noise to translation
  pose_j = pose_init(imu_buf.ts(end), T_WS_j);

  % Keep track of ground truth pose
  poses_gnd{pose_idx} = sim_data.poses{end_idx};
  pose_idx += 1;

  % Keep track of ground truth velocities
  vels_gnd{vel_idx} = sim_data.vel(:, end_idx);
  vel_idx += 1;

  % Speed and bias j
  vel_j = sim_data.vel(:, end_idx);
  vel_j += normrnd(0.0, 0.5, 3, 1); % Add noise to velocities
  ba_j = zeros(3, 1);
  bg_j = zeros(3, 1);
  sb_j = sb_init(imu_buf.ts(end), vel_j, bg_j, ba_j);

  % Add imu factor
  [graph, pose_j_id] = graph_add_param(graph, pose_j);
  [graph, sb_j_id] = graph_add_param(graph, sb_j);
  param_ids = [pose_i_id; sb_i_id; pose_j_id; sb_j_id];
  imu_factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i);
  graph = graph_add_factor(graph, imu_factor);

  % Update
  pose_i = pose_j;
  pose_i_id = pose_j_id;
  sb_i = sb_j;
  sb_i_id = sb_j_id;
endfor


% Visualize
visualize("Before optimization", graph, poses_gnd, vels_gnd);

% Optimize
max_iter = 5;
for i = 1:max_iter
  [H, g, r, param_idx] = graph_eval(graph);
  H = H + 1e-6 * eye(size(H)); % Levenberg-Marquardt Dampening
  dx = linsolve(H, g);

  graph = graph_update(graph, param_idx, dx);
  cost = 0.5 * r' * r
end

visualize("After Optimization", graph, poses_gnd, vels_gnd);
ginput();
