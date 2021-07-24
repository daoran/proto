function sim_data = sim_vio(circle_r, velocity)
  imu_data = sim_imu(circle_r, velocity);
  cam_data = sim_vo(circle_r, velocity);

  sim_data = {};
  % -- Features
  sim_data.nb_features = cam_data.nb_features;
  sim_data.features    = cam_data.features;
  % -- Camera
  sim_data.cam0       = cam_data.cam0;
  sim_data.cam_ts     = cam_data.cam_ts;
  sim_data.cam_poses  = cam_data.cam_poses;
  sim_data.cam_pos    = cam_data.cam_pos;
  sim_data.cam_quat   = cam_data.cam_quat;
  sim_data.cam_att    = cam_data.cam_att;
  sim_data.cam_z_data = cam_data.cam_z_data;
  sim_data.cam_p_data = cam_data.cam_p_data;
  % -- IMU
  sim_data.imu_poses = imu_data.imu_poses;
  sim_data.imu_pos   = imu_data.imu_pos;
  sim_data.imu_quat  = imu_data.imu_quat;
  sim_data.imu_att   = imu_data.imu_att;
  sim_data.imu_vel   = imu_data.imu_vel;
  sim_data.imu_time  = imu_data.imu_time;
  sim_data.imu_acc   = imu_data.imu_acc;
  sim_data.imu_gyr   = imu_data.imu_gyr;
endfunction
