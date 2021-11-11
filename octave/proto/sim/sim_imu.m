function sim_data = sim_imu(circle_r, velocity)
  imu_rate = 200.0;
  circle_dist = 2.0 * pi * circle_r
  time_taken = circle_dist / velocity
  g = [0.0; 0.0; 9.81];
  printf("Simulating ideal IMU measurements ...\n");
  printf("imu_rate: %f\n", imu_rate);
  printf("circle_r: %f\n", circle_r);
  printf("circle_dist: %f\n", circle_dist);
  printf("time_taken: %f\n", time_taken);

  dt = 1.0 / imu_rate;
  w = -2.0 * pi * (1.0 / time_taken)
  t = 0;

  theta = pi;
  yaw = pi / 2.0;

  sensor_poses = {};
  sensor_pos = [];
  sensor_quat = [];
  sensor_att = [];
  sensor_vel = [];

  imu_time = [];
  imu_acc = [];
  imu_gyr = [];

  idx = 1;
  while (t <= time_taken)
    % Sensor pose
    rx = circle_r * cos(theta);
    ry = circle_r * sin(theta);
    rz = 0.0;
    r_WS = [rx; ry; rz];
    rpy_WS = [0.0; 0.0; yaw];
    C_WS = euler321(rpy_WS);
    T_WS = tf(C_WS, r_WS);

    % Sensor velocity
    vx = -circle_r * w * sin(theta);
    vy = circle_r * w * cos(theta);
    vz = 0.0;
    v_WS = [vx; vy; vz];

    % Sensor acceleration
    ax = -circle_r * w * w * cos(theta);
    ay = -circle_r * w * w * sin(theta);
    az = 0.0;
    a_WS = [ax; ay; az];

    % Sensor angular velocity
    wx = 0.0;
    wy = 0.0;
    wz = w;
    w_WS = [wx; wy; wz];

    % IMU measurements
    acc = C_WS' * (a_WS + g);
    gyr = C_WS' * w_WS;

    % Update
    sensor_poses{idx} = T_WS;
    sensor_pos = [sensor_pos, tf_trans(T_WS)];
    sensor_quat = [sensor_quat, tf_quat(T_WS)];
    sensor_att = [sensor_att, quat2euler(tf_quat(T_WS))];
    sensor_vel = [sensor_vel, v_WS];

    imu_time = [imu_time; t];
    imu_acc = [imu_acc, acc];
    imu_gyr = [imu_gyr, gyr];

    idx += 1;
    theta += w * dt;
    yaw += w * dt;
    t += dt;

    % if (t >= (time_taken * 0.3))
    %   break;
    % end
  endwhile

  sim_data = {};
  sim_data.imu_poses = sensor_poses;
  sim_data.imu_pos = sensor_pos;
  sim_data.imu_quat = sensor_quat;
  sim_data.imu_att = sensor_att;
  sim_data.imu_vel = sensor_vel;
  sim_data.imu_time = imu_time;
  sim_data.imu_acc = imu_acc;
  sim_data.imu_gyr = imu_gyr;
endfunction
