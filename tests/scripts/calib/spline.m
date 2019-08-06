#!/usr/bin/octave -qf
graphics_toolkit("fltk");

arg_list = argv();
points_csv = arg_list{1}
spline_csv = arg_list{2}
spline_deriv_csv = arg_list{3}
pts_data = csvread(points_csv, 0, 0);
spl_data = csvread(spline_csv, 0, 0);
spl_deriv_data = csvread(spline_deriv_csv, 0, 0);


figure();

subplot(211);
hold on;
plot(pts_data(:, 1), pts_data(:, 2), "r.", "markersize", 20);
plot(spl_data(:, 1), spl_data(:, 2), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Response");
title("Signal");

subplot(212);
hold on;
plot(spl_deriv_data(:, 1), spl_deriv_data(:, 2), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Response");
title("Signal Derivative");

ginput()
