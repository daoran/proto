#!/usr/bin/octave -qf

arg_list = argv();
csv_path = arg_list{1};

data = csvread(csv_path, 1, 0);
t = data(:, 1);
r = data(:, 2);
p = data(:, 3);
y = data(:, 4);

figure();
hold on;
plot(t, r, "r-", "linewidth", 2.0);
plot(t, p, "g-", "linewidth", 2.0);
plot(t, y, "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Degree [rad]");
legend("Roll", "Pitch", "Yaw");
ginput();
