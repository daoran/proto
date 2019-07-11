addpath(genpath("."));
graphics_toolkit("fltk");
pkg load statistics;

function data = synchronize_measurements(timeline)
  % Interpolate data as it arrives
  cam0_type = 0;
  cam1_type = 1;
  accel0_type = 2;
  gyro0_type = 3;

  accel0_started = false;
  gyro0_started = false;
  imu_started = false;

  accel0_t0 = {};
  gyro0_t0 = {};

  cam0_ts = [];
  cam1_ts = [];
  accel0_ts = [];
  accel0_data = [];
  gyro0_ts = [];
  gyro0_data = [];

  interp_buf = [];

  for i = 1:length(timeline)
  % for i = 1:10
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
        if accel0_started == false
          accel0_started = true;
        endif

        % Add to buffer
        accel0_ts = [accel0_ts, ts];
        accel0_data = [accel0_data, timeline(i).accel0];
      endif

      % Gyro event handler
      if event_type(j) == gyro0_type
        if gyro0_started == false
          gyro0_started = true;
        endif

        % Add to buffer
        gyro0_ts = [gyro0_ts, ts];
        gyro0_data = [gyro0_data, timeline(i).gyro0];
      endif

			% Check if we got the first gyro and accel measurement
      if gyro0_started && accel0_started
        imu_started = true;
      endif
    endfor

    % Add interp ts
    if gyro0_ts(end) > accel0_ts(end)
      interp_buf = [interp_buf; ts];
    endif

    % Interpolate accel0
    if length(interp_buf) && (accel0_ts(end) > interp_buf(end))
      % Interpolation start point
      accel0_t0 = {};
      accel0_t0.ts = accel0_ts(end-1);
      accel0_t0.data = accel0_data(:, end-1);

      % Interpolation end point
      accel0_t1 = {};
      accel0_t1.ts = accel0_ts(end);
      accel0_t1.data = accel0_data(:, end);

      % Remove last accel0 timestamp and data
      accel0_ts = accel0_ts(1:end-1);
      accel0_data = accel0_data(:, 1:end-1);

      % Interpolate
      for i = 1:length(interp_buf)
        interp_ts = interp_buf(i);

        % Lerp
        if (interp_ts - accel0_t0.ts) > 0
          t = (interp_ts - accel0_t0.ts) / (accel0_t1.ts - accel0_t0.ts);
          accel0_lerped = lerp(accel0_t0.data, accel0_t1.data, t);
        else
          accel0_lerped = accel0_t0.data;
        endif

        % Add interpolated point to buffer
        accel0_ts = [accel0_ts, interp_ts];
        accel0_data = [accel0_data, accel0_lerped];
      endfor

      accel0_ts = [accel0_ts, accel0_t1.ts];
      accel0_data = [accel0_data, accel0_t1.data];

      % Reset interpolation buffer
      interp_buf = [];
    endif
  endfor

  % Form return
  data = {};
  data.cam0_ts = cam0_ts;
  data.cam1_ts = cam1_ts;
  data.accel0_ts = accel0_ts;
  data.accel0_data = accel0_data;
  data.gyro0_ts = gyro0_ts;
  data.gyro0_data = gyro0_data;

	% length(accel0_ts)
	% length(gyro0_ts)
	% assert(length(accel0_ts) == length(gyro0_ts));
	% assert(length(accel0_data) == length(gyro0_data));
endfunction


% MAIN
timeline = timeline_create();
data = synchronize_measurements(timeline);

% Plot timestamps
figure(1);
subplot(211);
plot_timestamps("Un-Synchronized Measurements",
                cam0_ts = timeline_flatten_cam0_ts(timeline),
                cam1_ts = timeline_flatten_cam1_ts(timeline),
                accel0_ts = timeline_flatten_accel0_ts(timeline),
                gyro0_ts = timeline_flatten_gyro0_ts(timeline))
subplot(212);
plot_timestamps("Synchronized Measurements",
                data.cam0_ts,
                data.cam1_ts,
                data.accel0_ts,
                data.gyro0_ts);
ginput();

% Plot gyroscope measurements
figure(2);
subplot(211);
gyro0_ts = timeline_flatten_gyro0_ts(timeline);
gyro0_data = timeline_flatten_gyro0_data(timeline);
plot_measurements("Gyroscope Data",
								  gyro0_ts,
									gyro0_data,
								  "rad s^-1");
subplot(212);
gyro0_ts = data.gyro0_ts;
gyro0_data = data.gyro0_data;
plot_measurements("Interpolated Gyroscope Data",
									  gyro0_ts,
								    gyro0_data,
									  "rad s^-1");
ginput();

% Plot accelerometer measurements
figure(3);
subplot(211);
accel0_ts = timeline_flatten_accel0_ts(timeline);
accel0_data = timeline_flatten_accel0_data(timeline);
yunit = "ms^{-1}";
plot_measurements("Accelerometer Data", accel0_ts, accel0_data, yunit);
subplot(212);
accel0_ts = data.accel0_ts;
accel0_data = data.accel0_data;
plot_measurements("Interpolated Accelerometer Data", accel0_ts, accel0_data, yunit);
ginput();
