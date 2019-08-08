#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
att_csv = arg_list{1}
att_prop_csv = arg_list{2}

% Parse data
att_data = csvread(att_csv, 0, 0);
att_prop_data = csvread(att_prop_csv, 0, 0);

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

subplot(211);
hold on;
t = att_data(:, 1) * 1e-9;

rpy_data = zeros(rows(att_data), 3);
for i = 1:rows(att_data)
  rpy = quat2euler(att_data(i, 2:end));
  assert(columns(rpy) == 1);
  rpy_data(i, :) = rpy';
endfor

plot(t, rpy_data(:, 1), "r.", "markersize", 10);
plot(t, rpy_data(:, 2), "b.", "markersize", 10);
plot(t, rpy_data(:, 3), "g.", "markersize", 10);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Attitude [rad]");
xlim([0, max(t)]);
title("Attitude Data points");

subplot(212);
hold on;
t = att_prop_data(:, 1) * 1e-9;

rpy_data = zeros(rows(att_prop_data), 3);
for i = 1:rows(att_prop_data)
  rpy = quat2euler(att_prop_data(i, 2:end));
  assert(columns(rpy) == 1);
  rpy_data(i, :) = rpy';
endfor

plot(t, rpy_data(:, 1), "r.", "markersize", 10);
plot(t, rpy_data(:, 2), "b.", "markersize", 10);
plot(t, rpy_data(:, 3), "g.", "markersize", 10);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Attitude [rad]");
xlim([0, max(t)]);
title("Propagated Attitude");

ginput()
