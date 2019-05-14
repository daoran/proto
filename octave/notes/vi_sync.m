addpath(genpath("prototype"));
graphics_toolkit("fltk");
pkg load statistics;


function x = lerp(x0, x1, t)
  x = (1 - t) * x0 + t * x1;
endfunction

function timeline = create_timeline()
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



% Interpolate data as it arrives
data = {};

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


timeline = create_timeline();
for i = 1:length(timeline)
% for i = 1:10
	ts = timeline(i).ts;
	event_type = timeline(i).type;

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

        accel0_t0 = {};
        accel0_t0.ts = ts;
        accel0_t0.data = timeline(i).accel0;
			endif

      % Interpolate accel0
      for i = 1:length(interp_buf)
        interp_ts = interp_buf(i);

        % Set lerp end point
        accel0_t1 = {};
        accel0_t1.ts = ts;
        accel0_t1.data = timeline(i).accel0;

        % Lerp
        if (interp_ts - accel0_t0.ts) > 0
          t = (interp_ts - accel0_t0.ts) / (accel0_t1.ts - accel0_t0.ts);
          accel0_lerped = lerp(accel0_t0.data, accel0_t1.data, t);
        else
          accel0_lerped = accel0_t0.data;
        endif

        % Shift end lerp point to start
        accel0_t0 = accel0_t1;

        % Add to buffer
        accel0_ts = [accel0_ts, interp_ts];
        accel0_data = [accel0_data, accel0_lerped];
      endfor

      % Add to buffer
      accel0_ts = [accel0_ts, ts];
      accel0_data = [accel0_data, timeline(i).accel0];
		endif

    % Gyro event handler
		if event_type(j) == gyro0_type
			if gyro0_started == false
				gyro0_started = true;
			endif

			interp_buf = [interp_buf, ts];

      % Add to buffer
      gyro0_ts = [gyro0_ts, ts];
      gyro0_data = [gyro0_data, timeline(i).gyro0];
    endif

		if gyro0_started && accel0_started
			imu_started = true;
		endif
	endfor
endfor



figure(1);
subplot(211);

cam0_plot = 0.25;
cam1_plot = 0.2;
accel0_plot = 0.15;
gyro0_plot = 0.10;

cam0_y = cam0_plot * ones(length(cam0_ts), 1);
cam1_y = cam1_plot * ones(length(cam1_ts), 1);
accel0_y = accel0_plot * ones(length(accel0_ts), 1);
gyro0_y = gyro0_plot * ones(length(gyro0_ts), 1);

hold on;
plot(cam0_ts, cam0_y, "rs", "linewidth", 2.0, "markerfacecolor", "r");
plot(cam1_ts, cam1_y, "rs", "linewidth", 2.0, "markerfacecolor", "r");
plot(accel0_ts, accel0_y, "bs", "linewidth", 2.0, "markerfacecolor", "b");
plot(gyro0_ts, gyro0_y, "gs", "linewidth", 2.0, "markerfacecolor", "g");
hold off;

sensors = {"gyro0", "accel0", "cam1", "cam0"};
y_tick = [gyro0_plot, accel0_plot, cam1_plot, cam0_plot];
set(gca, "ytick", y_tick, "yticklabel", sensors);
xlim([0, 0.12]);
ylim([0, 0.3]);
xlabel("Time [s]");
ylabel("Sensors");


subplot(212);

hold on;
size(gyro0_ts)
size(gyro0_data)
plot(gyro0_ts, gyro0_data(1, :), "r-", "linewidth", 2.0);

hold off;

ginput();


% % Plot timeline
% figure(1);
%
% cam0_plot = 0.75;
% cam1_plot = 0.7;
% accel0_plot = 0.25;
% gyro0_plot = 0.2;
%
% cam0_y = cam0_plot * ones(length(cam0_ts), 1);
% cam1_y = cam1_plot * ones(length(cam1_ts), 1);
% accel0_y = accel0_plot * ones(length(accel0_ts), 1);
% gyro0_y = gyro0_plot * ones(length(gyro0_ts), 1);
%
% hold on;
% plot(cam0_ts, cam0_y, "ro", "linewidth", 2.0);
% plot(cam1_ts, cam1_y, "ro", "linewidth", 2.0);
% plot(accel0_ts, accel0_y, "bo", "linewidth", 2.0);
% plot(gyro0_ts, gyro0_y, "go", "linewidth", 2.0);
% hold off;
%
% sensors = {"gyro0", "accel0", "cam1", "cam0"};
% y_tick = [gyro0_plot, accel0_plot, cam1_plot, cam0_plot];
% set(gca, "ytick", y_tick, "yticklabel", sensors);
% xlim([0, 0.2]);
% ylim([0, 1.0]);
% xlabel("Time [s]");
% ylabel("Sensors");
%
% ginput();
