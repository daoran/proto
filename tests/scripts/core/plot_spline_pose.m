#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
pos_csv = arg_list{1};
att_csv = arg_list{2};
pos_spline_csv = arg_list{3};
att_spline_csv = arg_list{4};

% Parse data
pos_data = csvread(pos_csv, 0, 0);
att_data = csvread(att_csv, 0, 0);
pos_spline_data = csvread(pos_spline_csv, 0, 0);
att_spline_data = csvread(att_spline_csv, 0, 0);

function euler = quat2euler(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qw2 = qw**2;
  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;

  t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  t2 = asin(2 * (qy * qw - qx * qz));
  t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));
  euler = [t1; t2; t3];
endfunction

% Plot figure
figure();

subplot(221);
hold on;
t = pos_data(:, 1) * 1e-9;
plot(t, pos_data(:, 2), "r.", "markersize", 20);
plot(t, pos_data(:, 3), "b.", "markersize", 20);
plot(t, pos_data(:, 4), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(t)]);
title("Position Data points");

subplot(222);
hold on;
t = pos_spline_data(:, 1) * 1e-9;
plot(t, pos_spline_data(:, 2), "r.", "markersize", 20);
plot(t, pos_spline_data(:, 3), "b.", "markersize", 20);
plot(t, pos_spline_data(:, 4), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(t)]);
title("Interpolated Position");

subplot(223);
hold on;
t = att_data(:, 1) * 1e-9;

rpy_data = zeros(rows(att_data), 3);
for i = 1:rows(att_data)
  rpy = quat2euler(att_data(i, 2:end));
  assert(columns(rpy) == 1);
  rpy_data(i, :) = rpy';
endfor

plot(t, rpy_data(:, 1), "r.", "markersize", 20);
plot(t, rpy_data(:, 2), "b.", "markersize", 20);
plot(t, rpy_data(:, 3), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Attitude [rad]");
xlim([0, max(t)]);
title("Attitude Data points");

subplot(224);
hold on;
t = att_spline_data(:, 1) * 1e-9;

rpy_data = zeros(rows(att_spline_data), 3);
for i = 1:rows(att_spline_data)
  rpy = quat2euler(att_spline_data(i, 2:end));
  assert(columns(rpy) == 1);
  rpy_data(i, :) = rpy';
endfor

plot(t, rpy_data(:, 1), "r.", "markersize", 20);
plot(t, rpy_data(:, 2), "b.", "markersize", 20);
plot(t, rpy_data(:, 3), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Attitude [rad]");
xlim([0, max(t)]);
title("Interpolated Attitude");

ginput()
