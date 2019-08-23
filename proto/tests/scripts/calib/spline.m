#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
points_csv = arg_list{1}
spline_csv = arg_list{2}
spline_vel_csv = arg_list{3}
spline_acc_csv = arg_list{4}

% Parse data
pts_data = csvread(points_csv, 0, 0);
spl_data = csvread(spline_csv, 0, 0);
spl_vel_data = csvread(spline_vel_csv, 0, 0);
spl_acc_data = csvread(spline_acc_csv, 0, 0);


% Plot figure
figure();

subplot(311);
hold on;
plot(pts_data(:, 1), pts_data(:, 2), "r.", "markersize", 20);
plot(spl_data(:, 1), spl_data(:, 2), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Response");
xlim([0, max(pts_data(:, 1))]);
title("Signal");

subplot(312);
hold on;
plot(spl_vel_data(:, 1), spl_vel_data(:, 2), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Response");
xlim([0, max(spl_vel_data(:, 1))]);
title("Signal Velocity");

subplot(313);
hold on;
plot(spl_acc_data(:, 1), spl_acc_data(:, 2), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Response");
xlim([0, max(spl_acc_data(:, 1))]);
title("Signal Acceleration");

ginput()
