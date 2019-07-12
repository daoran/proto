function data = visync(timeline)
  % Interpolate data as it arrives
  cam0_type = 0;
  cam1_type = 1;
  accel0_type = 2;
  gyro0_type = 3;

  % Flags
  accel0_started = false;
  gyro0_started = false;
  imu_started = false;

  % Buffer
  interp_buf = [];

  % Camera
  cam0_ts = [];
  cam1_ts = [];

  % Accelerometer
  accel = {};
  accel.started = false;
  accel.ts = [];
  accel.data = [];

  % Gyroscope
  gyro = {};
  gyro.started = false;
  gyro.ts = [];
  gyro.data = [];

  % Synchronize measurements
  for i = 1:length(timeline)
    ts = timeline(i).ts;
    event_type = timeline(i).type;

    % Iterate over events at the same timestamp
    for j = 1:length(event_type)
      % cam0 event handler
      if event_type(j) == cam0_type && imu_started
        cam0_ts = [cam0_ts, ts];
      endif

      % cam1 event handler
      if event_type(j) == cam1_type && imu_started
        cam1_ts = [cam1_ts, ts];
      endif

      % Accel event handler
      if event_type(j) == accel0_type
        if accel.started == false
          accel.started = true;
        endif

        % Add to buffer
        accel.ts = [accel.ts, ts];
        accel.data = [accel.data, timeline(i).accel0];
      endif

      % Gyro event handler
      if event_type(j) == gyro0_type
        % Interpolate gyro data
        if imu_started && (gyro.ts(end) < accel.ts(end))
          t0 = gyro.ts(end);
          t1 = ts;
          interp_ts = accel.ts(end);
          t = (interp_ts - t0) / (t1 - t0);
          if t < 1.0
            gyro_lerped = lerp(gyro.data(:, end), timeline(i).gyro0, t);
            gyro.ts = [gyro.ts, interp_ts];
            gyro.data = [gyro.data, gyro_lerped];
          endif
        endif

        if gyro.started == false
          gyro.started = true;
        endif

        % Add to buffer
        gyro.ts = [gyro.ts, ts];
        gyro.data = [gyro.data, timeline(i).gyro0];
      endif

			% Check if we got the first gyro and accel measurement
      if gyro.started && accel.started
        imu_started = true;
      endif
    endfor

    % Add interp ts
    if gyro.ts(end) > accel.ts(end)
      if length(interp_buf) && (gyro.ts(end) != interp_buf(end))
        interp_buf = [interp_buf; gyro.ts(end)];
      elseif length(interp_buf) == 0
        interp_buf = [interp_buf; gyro.ts(end)];
      endif
    endif

    % Interpolate
    if length(interp_buf) && (accel.ts(end) > gyro.ts(end))
      accel = lerp_batch(interp_buf, accel);
      interp_buf = [];
    endif
  endfor

  % Interpolate whats left in the buffer
  accel = lerp_batch(interp_buf, accel);
  [accel, gyro] = snip_ends(accel, gyro);

  % Form return
  data = {};
  data.cam0_ts = cam0_ts;
  data.cam1_ts = cam1_ts;
  data.accel0 = accel;
  data.gyro0 = gyro;
endfunction
