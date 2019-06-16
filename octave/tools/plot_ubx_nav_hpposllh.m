#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

pkg load statistics;
% graphics_toolkit("fltk");

script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

% hpposllh_csv = "/data/ubx_nav_hpposllh.csv";
hpposllh_csv = "/data/rtkgps/hyde_park-gps_only.csv";
% hpposllh_csv = "/data/rtkgps/hyde_park-static_1.csv";
% hpposllh_csv = "/data/rtkgps/hyde_park-static_2.csv";
% hpposllh_csv = "/data/rtkgps/hyde_park-static_all.csv";
% hpposllh_csv = "/data/rtkgps/hyde_park-circle_1.csv";
% hpposllh_csv = "/data/rtkgps/hyde_park-circle_2.csv";
% hpposllh_csv = "/data/rtkgps/imperial-circle_1.csv";
% hpposllh_csv = "/data/rtkgps/imperial-circle_2.csv";

[itow, lon, lat, height, hmsl, lon_hp, lat_hp, height_hp, hmsl_hp, hacc, vacc] = textread(
% data = textread(
  hpposllh_csv,
  "%d %d %d %d %d %d %d %d %d %d %d",
  "delimiter", ",",
  "headerlines", 1
);

% Convert integers to double
itow = double(itow) * 1e-3;
lon = (double(lon) * 1e-7) + (double(lon_hp) * 1e-9);
lat = (double(lat) * 1e-7) + (double(lat_hp) * 1e-9);
height = (double(height) * 1e-3) + (double(height_hp) * 1e-4);

% Grab reference time, lon, lat and height
itow_ref = itow(1);
t = itow - itow_ref;
lon_ref = lon(1);
lat_ref = lat(1);
height_ref = height(1);

% Convert lat, lon, altitude to cartesian
[x, y, z] = loccart_fwd(lat_ref, lon_ref, height_ref, lat, lon, height);

printf("Mean x [m]: %f\n", mean(x));
printf("Mean y [m]: %f\n", mean(y));
printf("Mean z [m]: %f\n", mean(z));
printf("Variance x [m]: %f\n", var(x));
printf("Variance y [m]: %f\n", var(y));
printf("Variance z [m]: %f\n", var(z));

% % Birds Eye View Plot - For circle trajectory
% fig = figure();
% hold on;
% plot(x, y, "r.");
% grid on;
% axis equal;
% xlabel("Displacement [m]");
% ylabel("Displacement [m]");
% print(fig, "trajectory.png")

% % Birds Eye View Plot
% fig = figure();
% hold on;
% plot(x, y, "r.");
% % -- 99% confidence level
% [r_ellipse, X0, Y0] = error_ellipse_2d(x, y, sqrt(9.21));
% plot(r_ellipse(:,1) + X0, r_ellipse(:,2) + Y0, 'b-')
% % -- 95% confidence level
% [r_ellipse, X0, Y0] = error_ellipse_2d(x, y, sqrt(5.99));
% plot(r_ellipse(:,1) + X0, r_ellipse(:,2) + Y0, 'b-')
% % -- 90% confidence level
% [r_ellipse, X0, Y0] = error_ellipse_2d(x, y, sqrt(4.61));
% plot(r_ellipse(:,1) + X0, r_ellipse(:,2) + Y0, 'b-')
% % -- Draw horizontal and vertical line around (0, 0)
% plot([0, 0], [-0.05, 0.05], "k-");
% plot([-0.05, 0.05], [0, 0], "k-");
% % -- Plot settings
% grid on;
% % axis equal;
% xlim([-0.04, 0.04]);
% ylim([-0.04, 0.04]);
% xlabel("lat [m]");
% ylabel("lon [m]");
% set(gca,'XTick',-0.04:0.01:0.04)
% set(gca,'YTick',-0.04:0.01:0.04)
% print(fig, "trajectory.png")

% Box-Plot
fig = figure();
axis([0, 3]);
boxplot({x, y, z});
set(gca (), "xtick", [1 2 3], "xticklabel", {"lat", "lon", "height"})
ylabel("Displacement [m]");
print(fig, "boxplot.png")

% Time-Series Plot
fig = figure();
hold on;
plot(t, x, "r-");
plot(t, y, "g-");
plot(t, z, "b-");
set(gca, "xaxislocation", "origin");
set(gca, "yaxislocation", "origin");
xlabel("Time [s]");
ylabel("Displacement [m]");
legend("x", "y", "z");
print(fig, "xyplot.png")

ginput();
