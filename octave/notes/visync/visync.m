function data = visync(timeline, plot=false)
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
  lerp_buf_accel = [];
  lerp_buf_gyro = [];

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
    if accel.ts(end) > gyro.ts(end)
      if length(lerp_buf_gyro) && (accel.ts(end) != lerp_buf_gyro(end))
        lerp_buf_gyro = [lerp_buf_gyro; accel.ts(end)];
      elseif length(lerp_buf_gyro) == 0
        lerp_buf_gyro = [lerp_buf_gyro; accel.ts(end)];
      endif
    endif
    if gyro.ts(end) > accel.ts(end)
      if length(lerp_buf_accel) && (gyro.ts(end) != lerp_buf_accel(end))
        lerp_buf_accel = [lerp_buf_accel; gyro.ts(end)];
      elseif length(lerp_buf_accel) == 0
        lerp_buf_accel = [lerp_buf_accel; gyro.ts(end)];
      endif
    endif

    % Interpolate
    if length(lerp_buf_gyro) && (gyro.ts(end) > accel.ts(end))
      gyro = lerp_batch(lerp_buf_gyro, gyro);
      lerp_buf_gyro = [];
    endif
    if length(lerp_buf_accel) && (accel.ts(end) > gyro.ts(end))
      accel = lerp_batch(lerp_buf_accel, accel);
      lerp_buf_accel = [];
    endif

    % Plot
    if plot == true
      data = {};
      data.cam0_ts = cam0_ts;
      data.cam1_ts = cam1_ts;
      data.accel0 = accel;
      data.gyro0 = gyro;
      compare_timestamps(timeline, data);
      ginput();
    endif
  endfor


  % Interpolate whats left in the buffer
  accel = lerp_batch(lerp_buf_accel, accel);
  [accel, gyro] = snip_ends(accel, gyro);
  assert(accel.ts(1) == gyro.ts(1));
  assert(accel.ts(end) == gyro.ts(end));
  assert(length(accel.ts) == length(gyro.ts));
  assert(length(accel.data) == length(gyro.data));

  % Form return
  data = {};
  data.cam0_ts = cam0_ts;
  data.cam1_ts = cam1_ts;
  data.accel0 = accel;
  data.gyro0 = gyro;
endfunction
