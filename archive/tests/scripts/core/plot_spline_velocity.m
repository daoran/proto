#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
pos_csv = arg_list{1};
vel_csv = arg_list{2};

% Parse data
pos_data = csvread(pos_csv, 0, 0);
vel_data = csvread(vel_csv, 0, 0);

% Plot figure
figure();

subplot(211);
hold on;
t = pos_data(:, 1) * 1e-9;
plot(t, pos_data(:, 2), "r.", "markersize", 20);
plot(t, pos_data(:, 3), "b.", "markersize", 20);
plot(t, pos_data(:, 4), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("x [m]");
xlim([0, max(t)]);
title("Position");

subplot(212);
hold on;
t = vel_data(:, 1) * 1e-9;
plot(t, vel_data(:, 2), "r.", "markersize", 20);
plot(t, vel_data(:, 3), "b.", "markersize", 20);
plot(t, vel_data(:, 4), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Velocity [ms^-1]");
xlim([0, max(t)]);
title("Velocity");

ginput()
