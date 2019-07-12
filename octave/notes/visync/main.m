addpath(genpath("."));
graphics_toolkit("fltk");
pkg load statistics;

% Create timeline and synchronize measurements
timeline = timeline_create();
data = visync(timeline, true);

% Plot
compare_timestamps(timeline, data);
% compare_gyro_measurements(timeline, data);
% compare_accel_measurements(timeline, data);
ginput();
