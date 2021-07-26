function sim_data = sim_vio(circle_r, velocity)
  imu_data = sim_imu(circle_r, velocity);
  cam_data = sim_vo(circle_r, velocity);

  % Form timeline
  timeline = [];
  imu_idx = 1;
  cam_idx = 1;

  while 1
    % Check which sensor measurement is available
    imu_time = imu_data.imu_time(imu_idx);
    cam_time = cam_data.cam_time(cam_idx);
    is_imu_event = 0;
    is_cam_event = 0;
    if abs(imu_time - cam_time) < 1e-4
      is_imu_event = 1;
      is_cam_event = 1;
    elseif cam_time > imu_time
      is_imu_event = 1;
      is_cam_event = 0;
    endif

    % Form timeline event
    event = {};
    % -- IMU data
    event.time = imu_time;
    event.has_imu_data = 1;
    event.imu_pose = imu_data.imu_poses{imu_idx};
    event.imu_vel = imu_data.imu_vel(:, imu_idx);
    event.imu_acc = imu_data.imu_acc(:, imu_idx);
    event.imu_gyr = imu_data.imu_gyr(:, imu_idx);
    % -- Camera data
    if is_cam_event
      event.has_cam_data = 1;
      event.cam_pose = cam_data.cam_poses{cam_idx};
      event.cam_z_data = cam_data.cam_z_data{cam_idx};
      event.cam_p_data = cam_data.cam_p_data{cam_idx};
    else
      event.has_cam_data = 0;
      event.cam_pose = 0;
      event.cam_z_data = 0;
      event.cam_p_data = 0;
    endif

    % Add to timeline
    timeline = [timeline, event];

    % Update indicies
    if abs(imu_time - cam_time) < 1e-4
      imu_idx += 1;
      cam_idx += 1;
    elseif cam_time > imu_time
      imu_idx += 1;
    endif

    % Break loop
    if (imu_idx > length(imu_data.imu_time)
        || (cam_idx > length(cam_data.cam_time)))
      break;
    endif
  endwhile

  sim_data = {};
  % -- Timeline
  sim_data.timeline    = timeline;
  % -- Features
  sim_data.nb_features = cam_data.nb_features;
  sim_data.features    = cam_data.features;
  % -- Camera
  sim_data.T_SC0      = cam_data.T_SC0;
  sim_data.cam0       = cam_data.cam0;
  sim_data.cam_time   = cam_data.cam_time;
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
