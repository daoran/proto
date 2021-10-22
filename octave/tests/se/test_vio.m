addpath(genpath("proto"));

function visualize(fig_title, graph, gnd_data)
  % Extract ground truth and estimate data
  t = [];
  pos_gnd = gnd_data.imu_pos;
  rpy_gnd = gnd_data.imu_att;
  v_gnd = gnd_data.imu_vel;
  pos_est = [];
  rpy_est = [];
  v_est = [];

  last_factor = 0;
  for k = 1:length(graph.factors)
    factor = graph.factors{k};
    if strcmp(factor.type, "imu_factor") == 0
      continue;
    endif

    params = graph_get_params(graph, factor.param_ids);
    pose_i = params{1};
    sb_i = params{2};
    T_WS = tf(pose_i.param);

    t = [t, factor.imu_buf.ts(end)];
    pos_est = [pos_est, tf_trans(T_WS)];
    rpy_est = [rpy_est, rad2deg(rot2euler(tf_rot(T_WS)))];
    v_est = [v_est, sb_i.param(1:3)];
    last_factor = factor;
  endfor

  % Extract last estimates
  params = graph_get_params(graph, factor.param_ids);
  pose_j = params{3};
  sb_j = params{4};
  T_WS = tf(pose_j.param);
  t = [t, factor.imu_buf.ts(end)];
  pos_est = [pos_est, tf_trans(T_WS)];
  rpy_est = [rpy_est, rad2deg(rot2euler(tf_rot(T_WS)))];
  v_est = [v_est, sb_j.param(1:3)];

  % Plot
  % | 1  | 2  | 3  | 4  | 5  | 6  | 7  |
  % | 8  | 9  | 10 | 11 | 12 | 13 | 14 |
  % | 15 | 16 | 17 | 18 | 19 | 20 | 21 |
  figure();
  title(fig_title);

  % -- X-Y Position
  subplot(3, 7, [1, 2, 3, 8, 9, 10, 15, 16, 17]);
  hold on;
  plot(pos_gnd(1, :), pos_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(pos_est(1, :), pos_est(2, :), 'r-', 'linewidth', 1.0);
  axis('equal');
  xlabel("Displacement in x-direction [m]");
  ylabel("Displacement in y-direction [m]");
  lgnd = legend("Ground Truth", "Estimate");
  legend boxoff;
  set(lgnd, 'color', 'none');
  title("X-Y Position");

  % -- Velocities
  subplot(3, 7, [4, 5]);
  hold on;
  plot(t, v_gnd(1, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(1, :), 'r-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  title("Velocity\nx-direction");

  subplot(3, 7, [11, 12]);
  hold on;
  plot(t, v_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(2, :), 'g-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  title("y-direction");

  subplot(3, 7, [18, 19]);
  hold on;
  plot(t, v_gnd(3, :), 'k-', 'linewidth', 2.0);
  plot(t, v_est(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  title("z-direction");

  % -- Attitude
  subplot(3, 7, [6, 7]);
  hold on;
  plot(t, rpy_gnd(1, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(1, :), 'r-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  title("Attitude\nRoll");

  subplot(3, 7, [13, 14]);
  hold on;
  plot(t, rpy_gnd(2, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(2, :), 'g-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  title("Pitch");

  subplot(3, 7, [20, 21]);
  hold on;
  plot(t, rpy_gnd(3, :), 'k-', 'linewidth', 2.0);
  plot(t, rpy_est(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  title("Yaw");

  pause(0.1);
  refresh();
endfunction

% Simulate imu data
sim_data = sim_vio(2.0, 10.0);
g = [0.0; 0.0; 9.81];

% IMU params
imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

% Create graph
graph = graph_init();
gnd_data = {};
gnd_data.imu_pos = [tf_trans(sim_data.imu_poses{1})];
gnd_data.imu_att = [rad2deg(quat2euler(tf_quat(sim_data.imu_poses{1})))];
gnd_data.imu_vel = [sim_data.imu_vel(:, 1)];
gnd_data.exts = sim_data.T_SC0;

% -- Add features
fid2pid = {};
for i = 1:rows(sim_data.features)
  p_W = sim_data.features(i, :)';
  [graph, param_id] = graph_add_param(graph, feature_init(i, p_W));
  fid2pid{i} = param_id;
endfor

% -- Add cam0
[graph, cam_id] = graph_add_param(graph, sim_data.cam0);

% -- Add imu-cam extrinsics
T_SC0 = sim_data.T_SC0;
T_SC0(1:3, 1:3) *= Exp(normrnd(0.0, 0.05, 3, 1)); % Add noise to rotation
T_SC0(1:3, 4) += normrnd(0.0, 0.1, 3, 1);         % Add noise to translation
exts = extrinsics_init(sim_data.imu_time(1), T_SC0);
[graph, exts_id] = graph_add_param(graph, exts);

% -- Add initial pose and speed and biases
% ---- Pose i
T_WS = sim_data.imu_poses{1};
pose_i = pose_init(sim_data.imu_time(1), T_WS);
[graph, pose_i_id] = graph_add_param(graph, pose_i);
% ---- Speed and bias i
vel_i = sim_data.imu_vel(:, 1);
ba_i = zeros(3, 1);
bg_i = zeros(3, 1);
sb_i = sb_init(sim_data.imu_time(1), vel_i, bg_i, ba_i);
[graph, sb_i_id] = graph_add_param(graph, sb_i);

% -- Add pose factor (Add pose factor to lock the first pose at origin)
pose_covar = 1e-8 * eye(6);
pose_factor = pose_factor_init(0, [pose_i_id], sim_data.imu_poses{1}, pose_covar);
graph = graph_add_factor(graph, pose_factor);

% Add cam factor for first frame
event = sim_data.timeline(1);
for i = 1:length(event.cam_p_data)
  fid = event.cam_p_data(i);
  feature_id = fid2pid{fid};
  z = event.cam_z_data(:, i);

  param_ids = [pose_i_id, exts_id, feature_id, cam_id];
  covar = 0.2^2 * eye(2);
  cam_factor = cam_factor_init(0, param_ids, z, covar);
  graph = graph_add_factor(graph, cam_factor);
endfor

% Initialize imu buffer
event = sim_data.timeline(1);
imu_buf = imu_buf_init(event.time, event.imu_acc, event.imu_gyr);

% Loop through time
for k = 2:length(sim_data.timeline)
  event = sim_data.timeline(k);
  t = event.time;

  % Add imu measurement to buffer
  imu_buf = imu_buf_add(imu_buf, t, event.imu_acc, event.imu_gyr);

  % Add factors
  if event.has_cam_data
    % Keep track of ground truth
    gnd_data.imu_pos = [gnd_data.imu_pos, tf_trans(event.imu_pose)];
    gnd_data.imu_att = [gnd_data.imu_att, rad2deg(quat2euler(tf_quat(event.imu_pose)))];
    gnd_data.imu_vel = [gnd_data.imu_vel, event.imu_vel];

    % Add pose
    T_WS_j = event.imu_pose;
    T_WS_j(1:3, 1:3) *= Exp(normrnd(0.0, 0.1, 3, 1)); % Add noise to rotation
    T_WS_j(1:3, 4) += normrnd(0.0, 0.3, 3, 1);         % Add noise to translation
    pose_j = pose_init(imu_buf.ts(end), T_WS_j);
    [graph, pose_j_id] = graph_add_param(graph, pose_j);

    % Add speed and bias
    vel_j = event.imu_vel;
    vel_j += normrnd(0.0, 0.5, 3, 1); % Add noise to velocities
    ba_j = zeros(3, 1);
    bg_j = zeros(3, 1);
    sb_j = sb_init(imu_buf.ts(end), vel_j, bg_j, ba_j);
    [graph, sb_j_id] = graph_add_param(graph, sb_j);

    % Add camera factor
    for i = 1:length(event.cam_p_data)
      fid = event.cam_p_data(i);
      feature_id = fid2pid{fid};
      z = event.cam_z_data(:, i);

      param_ids = [pose_j_id, exts_id, feature_id, cam_id];
      covar = 0.2^2 * eye(2);
      cam_factor = cam_factor_init(0, param_ids, z, covar);
      graph = graph_add_factor(graph, cam_factor);
    endfor

    % Add imu factor
    param_ids = [pose_i_id; sb_i_id; pose_j_id; sb_j_id];
    imu_factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i);
    graph = graph_add_factor(graph, imu_factor);

    % Update
    pose_i = pose_j;
    pose_i_id = pose_j_id;
    sb_i = sb_j;
    sb_i_id = sb_j_id;

    % Reset imu buffer
    imu_buf = imu_buf_reset(t, event.imu_acc, event.imu_gyr);
  endif
endfor


% Visualize
visualize("Before Optimization", graph, gnd_data);
print('-dpng', '-S1200,600', '/tmp/vio-before.png')
close;

% Optimize
T_SC0_est = tf(graph.params{exts_id}.param);
C_SC0_est = tf_rot(T_SC0_est);
rpy_SC0_est = rad2deg(rot2euler(C_SC0_est));
r_SC0_est = tf_trans(T_SC0_est);

printf("\n");
printf("Before optimization imu-cam extrinsics:\n");
printf("roll: %.4f, ", rpy_SC0_est(1));
printf("pitch: %.4f, ", rpy_SC0_est(2));
printf("yaw: %.4f, ", rpy_SC0_est(3));
printf("x: %.4f, ", r_SC0_est(1));
printf("y: %.4f, ", r_SC0_est(2));
printf("z: %.4f\n", r_SC0_est(3));
printf("\n");

graph = graph_solve(graph);

T_SC0_gnd = sim_data.T_SC0;
C_SC0_gnd = tf_rot(T_SC0_gnd);
rpy_SC0_gnd = rad2deg(rot2euler(C_SC0_gnd));
r_SC0_gnd = tf_trans(T_SC0_gnd);

T_SC0_est = tf(graph.params{exts_id}.param);
C_SC0_est = tf_rot(T_SC0_est);
rpy_SC0_est = rad2deg(rot2euler(C_SC0_est));
r_SC0_est = tf_trans(T_SC0_est);

printf("\n");
printf("Estimated imu-cam extrinsics:\n");
printf("roll: %.4f, ", rpy_SC0_est(1));
printf("pitch: %.4f, ", rpy_SC0_est(2));
printf("yaw: %.4f, ", rpy_SC0_est(3));
printf("x: %.4f, ", r_SC0_est(1));
printf("y: %.4f, ", r_SC0_est(2));
printf("z: %.4f\n", r_SC0_est(3));
printf("\n");

printf("Ground truth imu-cam extrinsics:\n");
printf("roll: %.4f, ", rpy_SC0_gnd(1));
printf("pitch: %.4f, ", rpy_SC0_gnd(2));
printf("yaw: %.4f, ", rpy_SC0_gnd(3));
printf("x: %.4f, ", r_SC0_gnd(1));
printf("y: %.4f, ", r_SC0_gnd(2));
printf("z: %.4f\n", r_SC0_gnd(3));
printf("\n");

visualize("After Optimization", graph, gnd_data);
print('-dpng', '-S1200,600', '/tmp/vio-after.png')
% ginput();
