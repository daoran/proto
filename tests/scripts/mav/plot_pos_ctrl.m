#!/usr/bin/octave -qf

arg_list = argv();
csv_path = arg_list{1};

data = csvread(csv_path, 1, 0);
t = data(:, 1);
px = data(:, 2);
py = data(:, 3);
pz = data(:, 4);
r = data(:, 5);
p = data(:, 6);
y = data(:, 7);

figure();
hold on;
plot(t, px, "r-", "linewidth", 2.0);
plot(t, py, "g-", "linewidth", 2.0);
plot(t, pz, "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Translation [m]");
legend("x", "y", "z");

figure();
hold on;
plot(t, r, "r-", "linewidth", 2.0);
plot(t, p, "g-", "linewidth", 2.0);
plot(t, y, "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Degree [rad]");
legend("Roll", "Pitch", "Yaw");

ginput();
