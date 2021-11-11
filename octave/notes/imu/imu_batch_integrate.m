function est = imu_batch_integrate(sim_data, method)
  printf("Performing batch IMU integration using [%s] method\n", method);

  % Initialize IMU state
  x_imu = imu_state_init();
  x_imu.p_WS = sim_data.imu_pos(:, 1);
  x_imu.v_WS = sim_data.imu_vel(:, 1);
  x_imu.C_WS = tf_rot(sim_data.imu_poses{1});

  % Batch integrate IMU measurements
  est_pos = [x_imu.p_WS];
  est_vel = [x_imu.v_WS];
  est_att = [rot2euler(x_imu.C_WS)];
  t_prev = sim_data.time(1);
  for k = 2:length(sim_data.time)
    % Calculate dt
    t = sim_data.time(k);
    dt = t - t_prev;

    % Propagate IMU state
    acc = sim_data.imu_acc(:, k);
    gyr = sim_data.imu_gyr(:, k);
    if strcmp(method, "euler") == 1
      x_imu = imu_euler_update(x_imu, acc, gyr, dt);
    elseif strcmp(method, "rk4") == 1
      x_imu = imu_rk4_update(x_imu, acc, gyr, dt);
    else
      printf("Invalid method [%s]!\n", method);
      exit(-1);
    endif

    % Keep track of t_prev
    est_pos = [est_pos, x_imu.p_WS];
    est_vel = [est_vel, x_imu.v_WS];
    est_att = [est_att, rot2euler(x_imu.C_WS)];
    t_prev = t;
  endfor

  est = {};
  est.time = sim_data.time;
  est.pos = est_pos;
  est.vel = est_vel;
  est.att = est_att;
endfunction
