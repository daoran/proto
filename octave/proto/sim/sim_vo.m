function sim_data = sim_vo(circle_r,
                           velocity,
                           C_SC0=euler321(deg2rad([-90.0, 0.0, -90.0])),
                           r_SC0 = [0.01; 0.01; 0.05])
  % cam0
  cam_idx = 0;
  image_width = 640;
  image_height = 480;
  resolution = [image_width; image_height];
  fov = 90.0;
  fx = focal_length(image_width, fov);
  fy = focal_length(image_width, fov);
  cx = image_width / 2;
  cy = image_height / 2;
  proj_params = [fx; fy; cx; cy];
  dist_params = [-0.01; 0.01; 1e-4; 1e-4];
  cam0 = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

  % Simulate features
  origin = [0; 0; 0];
  dim = [circle_r * 2; circle_r * 2; circle_r * 1.5];
  nb_features = 2000;
  features = create_3d_features_perimeter(origin, dim, nb_features);

  % Simulate camera
  cam_rate = 20.0;
  circle_dist = 2.0 * pi * circle_r;
  time_taken = circle_dist / velocity;

  % C_SC0 = euler321(deg2rad([-90.0, 0.0, -90.0]));
  % r_SC0 = [0.0; 0.0; 0.0];
  T_SC0 = tf(C_SC0, r_SC0);

  dt = 1.0 / cam_rate;
  w = -2.0 * pi * (1.0 / time_taken);
  t = 0;
  theta = pi;
  yaw = pi / 2.0;

  cam_time = [];
  cam_poses = {};
  cam_pos = [];
  cam_quat = [];
  cam_att = [];
  z_data = {};
  p_data = {};

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

    % Camera pose
    T_WC0 = T_WS * T_SC0;
    [z_data, p_data] = camera_measurements(cam0, T_WC0, features');
    cam_time = [cam_time; t];
    cam_poses{idx} = T_WC0;
    cam_pos = [cam_pos, tf_trans(T_WC0)];
    cam_quat = [cam_quat, tf_quat(T_WC0)];
    cam_att = [cam_att, quat2euler(tf_quat(T_WC0))];
    cam_z_data{idx} = z_data;
    cam_p_data{idx} = p_data;

    % Update
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
  % -- Camera
  sim_data.T_SC0 = T_SC0;
  sim_data.cam0 = cam0;
  sim_data.cam_time = cam_time;
  sim_data.cam_poses = cam_poses;
  sim_data.cam_pos = cam_pos;
  sim_data.cam_quat = cam_quat;
  sim_data.cam_att = cam_att;
  sim_data.cam_z_data = cam_z_data;
  sim_data.cam_p_data = cam_p_data;
endfunction
