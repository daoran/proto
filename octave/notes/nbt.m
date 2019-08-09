addpath(genpath("proto"));
graphics_toolkit("fltk");

% Setup calibration target
calib_target = calib_target_init(6, 7, 0.2);
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
T_WT = eye(4);
T_WT(1:3, 1:3) = C_WT;
T_WT(1:3, 4) = zeros(3, 1);

% Generate trajectories
idx = 1;
trajectories = {};

% -- Generate sphereical trajectories
rho = 1.0;
latitude_min = deg2rad(0.0);
latitude_max = deg2rad(360.0);
longitude_min = deg2rad(0.0);
longitude_max = deg2rad(90.0);
C_y = euler321([0.0, deg2rad(-90.0), 0.0]);

for latitude = linspace(latitude_min, latitude_max, 9)
  traj = [];
  for longitude = linspace(longitude_min, longitude_max, 10)
    p = C_y * sphere(rho, longitude, latitude) + [rho; 0.0; 0.0];
    traj = [traj, p];
  endfor
  trajectories{idx} = traj;
  idx++;
endfor

% -- Generate linear trajectories
theta_min = deg2rad(0.0);
theta_max = deg2rad(360.0);

for theta = linspace(theta_min, theta_max, 9)
  traj_start = [0.0; 0.0; 0.0];
  traj_end = circle(1.0, theta);
  traj_end = [0.0; traj_end];
  traj = [traj_start, traj_end];

  trajectories{idx} = traj;
  idx++;
endfor

% Plot scene
figure();
hold on;
for i = 1:length(trajectories)
  traj = trajectories{i};
  plot3(traj(1, :), traj(2, :), traj(3, :), "r-", "linewidth", 2);
  scatter3(traj(1, :), traj(2, :), traj(3, :), "r", "linewidth", 2);
  scatter3(traj(1, 1), traj(2, 1), traj(3, 1), "b", "linewidth", 2);
  scatter3(traj(1, end), traj(2, end), traj(3, end), "g", "linewidth", 2);
endfor
calib_target_draw(calib_target, T_WT);
view(3);
xlabel("x");
ylabel("y");
zlabel("z");
axis "equal";
ginput();
