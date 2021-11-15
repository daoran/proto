function save_sim_vio(sim_data, dir_path)
  mkdir(dir_path);

  % Save feature data
  features_path = strcat(dir_path, "/features.csv");
  csvwrite(features_path, sim_data.features);

  % Save imu data
  imu0_dir = strcat(dir_path, "/imu0");
  mkdir(imu0_dir);
  t = sim_data.imu_time;
  acc = sim_data.imu_acc';
  gyr = sim_data.imu_gyr';
  pos = sim_data.imu_pos';
  quat = sim_data.imu_quat';
  vel = sim_data.imu_vel';
  imu_data = [t, acc, gyr, pos, quat, vel];
  imu_data_path = strcat(imu0_dir, "/imu_data.csv");
  csvwrite(imu_data_path, imu_data);

  % Save cam data
  cam0_dir = strcat(dir_path, "/cam0");
  mkdir(strcat(cam0_dir, "/data"));

  % -- Save camera poses
  cam_poses = [];
  for k = 1:length(sim_data.cam_time)
    ts = sim_data.cam_time(k);
    T_WS = sim_data.cam_poses{k};
    cam_poses = [cam_poses; ts, tf_vector(T_WS)'];
  endfor
  save_path = strcat(cam0_dir, "/poses.csv");
  csvwrite(save_path, cam_poses);

  % -- Save camera frame data
  for k = 1:length(sim_data.cam_time)
    ts = sim_data.cam_time(k);
    save_path = strcat(cam0_dir, "/data/", num2str(ts * 1e9), ".csv");

    cam_data = [];
    for i = 1:length(sim_data.cam_z_data)
      z = sim_data.cam_z_data{k}(:, i);
      p = sim_data.cam_p_data{k}(i);
      cam_data = [cam_data; p, z(1), z(2)];
    endfor
    csvwrite(save_path, cam_data);
  endfor
endfunction
