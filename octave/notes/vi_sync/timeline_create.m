function timeline = timeline_create()
  % Sensor rates
  cam0_hz = 20.0;
  cam1_hz = 20.0;
  accel0_hz = 100.0;
  gyro0_hz = 400.0;

  % Create sensor timestamps
  t_end = 0.5;
  cam0_ts = 0:1.0/cam0_hz:t_end;
  cam1_ts = 0:1.0/cam0_hz:t_end;
  accel0_ts = 0:1.0/accel0_hz:t_end;
  gyro0_ts = 0:1.0/gyro0_hz:t_end;

  % Add noise to sensor timestamps
  add_noise = false;
  if add_noise
    % -- Camera
    cam_sigma = 0.001;
    for i = 2:length(cam0_ts)-1
      cam0_ts(i) = cam0_ts(i) + normrnd(0.0, cam_sigma);
    endfor
    for i = 2:length(cam1_ts)-1
      cam1_ts(i) = cam1_ts(i) + normrnd(0.0, cam_sigma);
    endfor
    % -- Accelerometer
    accel_sigma = 0.0001;
    for i = 2:length(accel0_ts)-1
      accel0_ts(i) = accel0_ts(i) + normrnd(0.0, accel_sigma);
    endfor
    % -- Gyroscope
    gyro_sigma = 0.0001;
    for i = 2:length(gyro0_ts)-1
      gyro0_ts(i) = gyro0_ts(i) + normrnd(0.0, gyro_sigma);
    endfor
  endif

  % Create timeline
  timeline = {};
  timeline_idx = 1;

  cam0_idx = 1;
  cam1_idx = 1;
  accel0_idx = 1;
  gyro0_idx = 1;

  cam0_type = 0;
  cam1_type = 1;
  accel0_type = 2;
  gyro0_type = 3;

  timestamps = unique([cam0_ts, cam1_ts, accel0_ts, gyro0_ts]);
  assert(min([cam0_ts, cam1_ts, accel0_ts, gyro0_ts]) == 0.0);
  assert(max([cam0_ts, cam1_ts, accel0_ts, gyro0_ts]) == t_end);

  for i = 1:length(timestamps)
    ts = timestamps(i);
    timeline(timeline_idx).ts = ts;
    timeline(timeline_idx).type = [];
    timeline(timeline_idx).accel0 = [];
    timeline(timeline_idx).gyro0 = [];

    % cam0
    if cam0_ts(cam0_idx) == ts
      timeline(timeline_idx).type = [timeline(timeline_idx).type, cam0_type];
      cam0_idx++;
    end
    % cam1
    if cam1_ts(cam1_idx) == ts
      timeline(timeline_idx).type = [timeline(timeline_idx).type, cam1_type];
      cam1_idx++;
    end
    % accel0
    if accel0_ts(accel0_idx) == ts
      timeline(timeline_idx).type = [timeline(timeline_idx).type, accel0_type];
      timeline(timeline_idx).accel0 = normrnd(0.0, 1.0, 3, 1);
      accel0_idx++;
    end
    % gyro0
    if gyro0_ts(gyro0_idx) == ts
      timeline(timeline_idx).type = [timeline(timeline_idx).type, gyro0_type];
      timeline(timeline_idx).gyro0 = normrnd(0.0, 1.0, 3, 1);
      gyro0_idx++;
    end

    timeline_idx++;
  endfor
endfunction
