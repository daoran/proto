function save_sim_vio(sim_data, dir_path)
  mkdir(dir_path);

  % Save feature data
  features_path = strcat(dir_path, "/features.csv");
  % csvwrite(features_path, sim_data.features);
  fid = fopen(features_path, "w");
  fprintf(fid, "#feature_id,x,y,z\n");
  for feature_id = 1:rows(sim_data.features)
    p = sim_data.features(feature_id, :);
    fprintf(fid, "%d,", feature_id);
    fprintf(fid, "%f,", p(1));
    fprintf(fid, "%f,", p(2));
    fprintf(fid, "%f\n", p(3));
  endfor
  fclose(fid);

  % Save imu data
  imu0_dir = strcat(dir_path, "/imu0");
  save_path = strcat(imu0_dir, "/data.csv");
  mkdir(imu0_dir);
  fid = fopen(save_path, "w");
  fprintf(fid, "#ts,px,py,pz,qx,qy,qz,qw,vx,vy,vz,ax,ay,az,wx,wy,wz\n");
  for k = 1:length(sim_data.imu_time)
    ts = num2str(sim_data.imu_time(k) * 1e9);
    acc = sim_data.imu_acc(:, k)';
    gyr = sim_data.imu_gyr(:, k)';
    pos = sim_data.imu_pos(:, k)';
    quat = sim_data.imu_quat(:, k)';
    vel = sim_data.imu_vel(:, k)';
    fprintf(fid, "%s,", ts);
    fprintf(fid, "%f,", pos(1));
    fprintf(fid, "%f,", pos(2));
    fprintf(fid, "%f,", pos(3));
    fprintf(fid, "%f,", quat(2));
    fprintf(fid, "%f,", quat(3));
    fprintf(fid, "%f,", quat(4));
    fprintf(fid, "%f,", quat(1));
    fprintf(fid, "%f,", vel(1));
    fprintf(fid, "%f,", vel(2));
    fprintf(fid, "%f,", vel(3));
    fprintf(fid, "%f,", acc(1));
    fprintf(fid, "%f,", acc(2));
    fprintf(fid, "%f,", acc(3));
    fprintf(fid, "%f,", gyr(1));
    fprintf(fid, "%f,", gyr(2));
    fprintf(fid, "%f\n", gyr(3));
  endfor
  fclose(fid);

  % Save cam data
  cam0_dir = strcat(dir_path, "/cam0");
  mkdir(strcat(cam0_dir, "/data"));

  % -- Save camera poses
  save_path = strcat(cam0_dir, "/data.csv");
  fid = fopen(save_path, "w");
  fprintf(fid, "#ts,px,py,pz,qx,qy,qz,qw\n");
  for k = 1:length(sim_data.cam_time)
    ts = num2str(sim_data.cam_time(k) * 1e9);
    pose = tf_vector(sim_data.cam_poses{k})';
    fprintf(fid, "%s,", ts);
    fprintf(fid, "%f,", pose(1));
    fprintf(fid, "%f,", pose(2));
    fprintf(fid, "%f,", pose(3));
    fprintf(fid, "%f,", pose(4));
    fprintf(fid, "%f,", pose(5));
    fprintf(fid, "%f,", pose(6));
    fprintf(fid, "%f\n", pose(7));
  endfor
  fclose(fid);

  % -- Save camera frame data
  for k = 1:length(sim_data.cam_time)
    ts = sim_data.cam_time(k);
    save_path = strcat(cam0_dir, "/data/", num2str(ts * 1e9), ".csv");

    fid = fopen(save_path, "w");
    fprintf(fid, "#feature_id,kp_x,kp_y\n");
    for i = 1:length(sim_data.cam_z_data)
      p = sim_data.cam_p_data{k}(i);
      z = sim_data.cam_z_data{k}(:, i);
      fprintf(fid, "%d,", p);
      fprintf(fid, "%f,", z(1));
      fprintf(fid, "%f\n", z(2));
    endfor
    fclose(fid);
  endfor
endfunction
