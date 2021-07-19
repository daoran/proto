function sim_data = sim_vio(circle_r, velocity)
  % cam0
  cam_idx = 0;
  image_width = 640;
  image_height = 480;
  resolution = [image_width; image_height];
  fov = 60.0;
  fx = focal_length(image_width, fov);
  fy = focal_length(image_height, fov);
  cx = image_width / 2;
  cy = image_height / 2;
  proj_params = [fx; fy; cx; cy];
  dist_params = [-0.01; 0.01; 1e-4; 1e-4];
  cam0 = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

  % Simulate features
  origin = [0; 0; 0];
  dim = [circle_r * 2; circle_r * 2; circle_r * 1.5];
  nb_features = 1000;
  features = create_3d_features_perimeter(origin, dim, nb_features);

  % Simulate imu
  imu_data = sim_imu(circle_r, velocity);

  % Simulate camera
  cam_rate = 20.0;
  circle_dist = 2.0 * pi * circle_r;
  time_taken = circle_dist / velocity;

  T_SC = tf(eye(3), [0.0; 0.0; 0.0]);

  dt = 1.0 / cam_rate;
  w = -2.0 * pi * (1.0 / time_taken);
  t = 0;
  theta = pi;
  yaw = 0;

  cam_poses = {};
  cam_measurements = {};

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

    % Update
    cam_poses{idx} = T_WS * T_SC;

    idx += 1;
    theta += w * dt;
    yaw += w * dt;
    t += dt;
  endwhile

  % Simulation data
  sim_data = {};
  % -- Features
  sim_data.nb_features = nb_features;
  sim_data.features = features;
  % -- IMU
  sim_data.sensor_poses = imu_data.poses;
  sim_data.sensor_vels = imu_data.vel;
  sim_data.imu_time = imu_data.time;
  sim_data.imu_acc = imu_data.imu_acc;
  sim_data.imu_gyr = imu_data.imu_gyr;
  % -- Camera
  sim_data.cam0 = cam0;
  sim_data.cam_poses = cam_poses;
  sim_data.cam_measurements = cam_measurements;
endfunction
