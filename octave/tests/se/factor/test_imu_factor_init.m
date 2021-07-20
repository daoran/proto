addpath(genpath("proto"));

sim_data = sim_imu(0.5, 1.0);
window_size = 10
g = [0.0; 0.0; 9.81];

i = 1;
start_idx = i;
end_idx = i + window_size - 1;

imu_buf = {};
imu_buf.ts = sim_data.time(start_idx:end_idx);
imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

param_ids = [1, 2, 3, 4];
sb_i = zeros(9, 1);
factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i)

%for i = 1:window_size:(length(sim_data.time)-window_size);
%  start_idx = i;
%  end_idx = i + window_size - 1;
%  imu_ts = sim_data.time(start_idx:end_idx);
%  imu_acc = sim_data.imu_acc(:, start_idx:end_idx);
%  imu_gyr = sim_data.imu_gyr(:, start_idx:end_idx);
%
%  pose_i = sim_data.poses{start_idx};
%  sb_i = [sim_data.vel(:, start_idx); 1e-2 * eye(6, 1)];
%  pose_j = sim_data.poses{end_idx};
%  sb_j = [sim_data.vel(:, end_idx); 3e-2 * eye(6, 1)];
%
%  check_imu_factor_jacobians(imu_ts, imu_acc, imu_gyr, g, pose_i, sb_i, pose_j, sb_j);
%  printf("\n\n");
%endfor
