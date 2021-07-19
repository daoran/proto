function factor = imu_factor_init(ts, param_ids, imu_buf, covar=eye(2))
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.imu_buf = imu_buf;
  factor.covar = covar;

  factor.noise_acc = 0.08;    % accelerometer measurement noise standard deviation.
  factor.noise_gyr = 0.004;   % gyroscope measurement noise standard deviation.
  factor.noise_ba = 0.00004;  % accelerometer bias random work noise standard deviation.
  factor.noise_bg = 2.0e-6;   % gyroscope bias random work noise standard deviation.

  factor.state_F = eye(15, 15);   % State jacobian
  factor.state_P = zeros(15, 15); % State covariance

  factor.Q = zeros(12, 12);
  factor.Q(1:3, 1:3) = (noise_acc * noise_acc) * eye(3);
  factor.Q(4:6, 4:6) = (noise_gyr * noise_gyr) * eye(3);
  factor.Q(7:9, 7:9) = (noise_ba * noise_ba) * eye(3);
  factor.Q(10:12, 10:12) = (noise_bg * noise_bg) * eye(3);

  % Pre-integrate relative position, velocity, rotation and biases
  factor.Dt = 0.0;
  dr = zeros(3, 1); % Relative position
  dv = zeros(3, 1); % Relative velocity
  dC = eye(3, 3);   % Relative rotation
  ba = sb_i(4:6);   % Accel biase
  bg = sb_i(7:9);   % Gyro biase

  % Pre-integrate imu measuremenets
  for k = 1:(length(imu_ts)-1)
    % Euler integration
    ts_i = imu_ts(k);
    ts_j = imu_ts(k+1);
    acc = imu_acc(:, k);
    gyr = imu_gyr(:, k);

    % Propagate IMU state using Euler method
    dt = ts_j - ts_i;
    dt_sq = dt * dt;
    dr = dr + (dv * dt) + (0.5 * dC * (acc - ba) * dt_sq);
    dv = dv + dC * (acc - ba) * dt;
    dC = dC * Exp((gyr - bg) * dt);
    ba = ba;
    bg = bg;

    % Continuous time transition matrix F
    F = zeros(15, 15);
    F(1:3, 4:6) = eye(3);
    F(4:6, 7:9) = -dC * skew(acc - ba);
    F(4:6, 10:12) = -dC;
    F(7:9, 7:9) = -skew(gyr - bg);
    F(7:9, 13:15) = -eye(3);

    % Continuous time input jacobian G
    G = zeros(15, 12);
    G(4:6, 1:3) = -dC;
    G(7:9, 4:6) = -eye(3);
    G(10:12, 7:9) = eye(3);
    G(13:15, 10:12) = eye(3);

    % Update
    G_dt = G * dt;
    I_F_dt = eye(15) + F * dt;
    factor.state_F = I_F_dt * factor.state_F;
    factor.state_P = I_F_dt * factor.state_P * I_F_dt' + G_dt * factor.Q * G_dt';
    factor.Dt += dt;
  endfor

  factor.dr = dr;
  factor.dv = dv;
  factor.dC = dC;
  factor.ba = ba;
  factor.bg = bg;
endfunction
