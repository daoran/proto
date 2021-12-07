function factor = imu_factor_init(param_ids, imu_buf, imu_params, sb_i, integration_type="midpoint")
  assert(length(param_ids) == 4);
  assert(strcmp(integration_type, "euler") || strcmp(integration_type, "midpoint"));

  factor.type = "imu_factor";
  factor.param_ids = param_ids;
  factor.imu_buf = imu_buf;
  factor.imu_params = imu_params;

  factor.integration_type = integration_type;
  factor.Dt = 0.0;
  factor.state_F = eye(15, 15);   % State jacobian
  factor.state_P = zeros(15, 15); % State covariance

  % Noise matrix Q
  if strcmp(integration_type, "euler")
    factor.Q = zeros(12, 12);
    factor.Q(1:3, 1:3) = (imu_params.noise_acc * imu_params.noise_acc) * eye(3);
    factor.Q(4:6, 4:6) = (imu_params.noise_gyr * imu_params.noise_gyr) * eye(3);
    factor.Q(7:9, 7:9) = (imu_params.noise_ba * imu_params.noise_ba) * eye(3);
    factor.Q(10:12, 10:12) = (imu_params.noise_bg * imu_params.noise_bg) * eye(3);
  elseif strcmp(integration_type, "midpoint")
    factor.Q = zeros(18, 18);
    factor.Q(1:3, 1:3) = (imu_params.noise_acc * imu_params.noise_acc) * eye(3);
    factor.Q(4:6, 4:6) = (imu_params.noise_gyr * imu_params.noise_gyr) * eye(3);
    factor.Q(7:9, 7:9) = (imu_params.noise_acc * imu_params.noise_acc) * eye(3);
    factor.Q(10:12, 10:12) = (imu_params.noise_gyr * imu_params.noise_gyr) * eye(3);
    factor.Q(13:15, 13:15) = (imu_params.noise_ba * imu_params.noise_ba) * eye(3);
    factor.Q(16:18, 16:18) = (imu_params.noise_bg * imu_params.noise_bg) * eye(3);
  endif

  % Pre-integrate relative position, velocity, rotation and biases
  dr = zeros(3, 1);        % Relative position
  dv = zeros(3, 1);        % Relative velocity
  dC = eye(3, 3);          % Relative rotation
  ba_i = sb_i.param(4:6);  % Accel biase at i
  bg_i = sb_i.param(7:9);  % Gyro biase at i

  % Pre-integrate imu measuremenets
  for k = 1:(length(imu_buf.ts)-1)
    % Timestep
    ts_i = imu_buf.ts(k);
    ts_j = imu_buf.ts(k+1);
    dt = ts_j - ts_i;
    dt_sq = dt * dt;

    % Accelerometer and gyroscope measurements
    acc_i = imu_buf.acc(:, k);
    gyr_i = imu_buf.gyr(:, k);
    acc_j = imu_buf.acc(:, k + 1);
    gyr_j = imu_buf.gyr(:, k + 1);

    if strcmp(integration_type, "euler")
      % Propagate IMU state using Euler method
      dr = dr + (dv * dt) + (0.5 * dC * (acc_i - ba_i) * dt_sq);
      dv = dv + dC * (acc_i - ba_i) * dt;
      dC = dC * Exp((gyr_i - bg_i) * dt);
      ba = ba_i;
      bg = bg_i;

      % Make sure determinant of rotation is 1 by normalizing the quaternion
      dq = quat_normalize(rot2quat(dC));
      dC = quat2rot(dq);

      % Continuous time transition matrix F
      F = zeros(15, 15);
      F(1:3, 4:6) = eye(3);
      F(4:6, 7:9) = -dC * skew(acc_i - ba_i);
      F(4:6, 10:12) = -dC;
      F(7:9, 7:9) = -skew(gyr_i - bg_i);
      F(7:9, 13:15) = -eye(3);

      % Continuous time input jacobian G
      G = zeros(15, 12);
      G(4:6, 1:3) = -dC;
      G(7:9, 4:6) = -eye(3);
      G(10:12, 7:9) = eye(3);
      G(13:15, 10:12) = eye(3);

    elseif strcmp(integration_type, "midpoint")
      % Propagate IMU state using midpoint method
      C_i = dC;
      r_i = dr;
      v_i = dv;

      C_j = C_i * Exp((0.5 * (gyr_i + gyr_j) - bg_i) * dt);
      r_j = r_i + (v_i * dt) + (0.5 * (0.5 * ((C_i * (acc_i - ba_i)) + (C_j * (acc_j - ba_i)))) * dt_sq);
      v_j = v_i + (0.5 * ((C_i * (acc_i - ba_i)) + (C_j * (acc_j - ba_i)))) * dt;
      ba_j = ba_i;
      bg_j = bg_i;

      dC = C_j;
      dr = r_j;
      dv = v_j;

      w_x = 0.5 * (gyr_i + gyr_j) - bg_i;
      a_i_x = acc_i - ba_i;
      a_j_x = acc_j - ba_i;
      C_w_x = skew(w_x);
      C_a_i_x = skew(a_i_x);
      C_a_j_x = skew(a_j_x);

      % Transition matrix F
      F = zeros(15, 15);
      % -- First row block
      F(1:3, 1:3) = eye(3);
      F(1:3, 4:6) = eye(3) * dt;
      F(1:3, 7:9) = -0.25 * C_i * C_a_i_x * dt_sq + -0.25 * C_j * C_a_j_x * (eye(3) - C_w_x * dt) * dt_sq;
      F(1:3, 10:12) = -0.25 * (C_i + C_j) * dt_sq;
      F(1:3, 13:15) = -0.25 * C_j * C_a_j_x * dt_sq * -dt;
      % -- Second row block
      F(4:6, 4:6) = eye(3);
      F(4:6, 7:9) = -0.5 * C_i * C_a_i_x * dt + -0.5 * C_j * C_a_i_x * (eye(3) - C_w_x * dt) * dt;
      F(4:6, 10:12) = -0.5 * (C_i + C_j) * dt;
      F(4:6, 13:15) = -0.5 * C_j * C_a_i_x * dt * -dt;
      % -- Third row block
      F(7:9, 7:9) = eye(3) - C_w_x * dt;
      F(7:9, 13:15) = -1.0 * eye(3) * dt;
      % -- Fourth row block
      F(10:12, 10:12) = eye(3);
      % -- Fifth row block
      F(13:15, 13:15) = eye(3);

      % Noise matrix G
      G = zeros(15, 18);
      % -- First row block
      G(1:3, 1:3) =  0.25 * C_i * dt_sq;
      G(1:3, 4:6) =  0.25 * -C_j * C_a_j_x  * dt_sq * 0.5 * dt;
      G(1:3, 7:9) =  0.25 * C_j * dt_sq;
      G(1:3, 10:12) =  G(1:3, 4:6);
      % -- Second row block
      G(4:6, 4:6) =  0.5 * eye(3) * dt;
      G(4:6, 10:12) =  0.5 * eye(3) * dt;
      % -- Third row block
      G(7:9, 1:3) =  0.5 * C_i * dt;
      G(7:9, 4:6) =  0.5 * -C_j * C_a_j_x  * dt * 0.5 * dt;
      G(7:9, 7:9) =  0.5 * C_j * dt;
      G(7:9, 10:12) =  G(7:9, 4:6);
      % -- Fourth row block
      G(10:12, 13:15) = eye(3) * dt;
      % -- Fifth row block
      G(13:15, 16:18) = eye(3) * dt;
    endif

    % Update
    G_dt = G * dt;
    I_F_dt = eye(15) + F * dt;
    factor.state_F = I_F_dt * factor.state_F;
    factor.state_P = I_F_dt * factor.state_P * I_F_dt' + G_dt * factor.Q * G_dt';
    factor.Dt += dt;
  endfor

  % figure()
  % % imagesc(factor.state_P)
  % imagesc(chol(inv(factor.state_P)))
  % colorbar()
  % ginput()

  % Update
  factor.dr = dr;
  factor.dv = dv;
  factor.dC = dC;
  factor.ba = ba_i;
  factor.bg = bg_i;
  factor.g = [0.0; 0.0; 9.81];
  factor.eval = @imu_factor_eval;
  factor.sqrt_info = chol(inv(factor.state_P));
endfunction
