addpath(genpath("proto"));
graphics_toolkit("fltk");

sim_vio(0.5, 1.0);
% fieldnames(sim_data)
%
% show_plots = 1;

% if show_plots
%   % Plot positions and velocities
%   figure();
%   subplot(311);
%   hold on;
%   plot(sim_data.time, sim_data.pos(1, :), 'r-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.pos(2, :), 'g-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.pos(3, :), 'b-', 'linewidth', 2.0);
%   xlabel("Time [s]");
%   ylabel("Displacement [m]");
%   legend('x', 'y', 'z');
%
%   subplot(312);
%   hold on;
%   plot(sim_data.time, sim_data.vel(1, :), 'r-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.vel(2, :), 'g-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.vel(3, :), 'b-', 'linewidth', 2.0);
%   xlabel("Time [s]");
%   ylabel("Velocity [ms^{-1}]");
%   legend('x', 'y', 'z');
%
%   subplot(313);
%   hold on;
%   plot(sim_data.pos(1, :), sim_data.pos(2, :), 'r-', 'linewidth', 2.0);
%   xlabel("Displacement [m]");
%   ylabel("Displacement [m]");
%   axis('equal');
%
%   % Plot accelerometer and gyroscope measurements
%   figure();
%   subplot(211);
%   hold on;
%   plot(sim_data.time, sim_data.imu_acc(1, :), 'r-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.imu_acc(2, :), 'g-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.imu_acc(3, :), 'b-', 'linewidth', 2.0);
%   xlabel("Time [s]");
%   ylabel("Acceleration [ms^{-2}]");
%   legend('x', 'y', 'z');
%
%   subplot(212);
%   hold on;
%   plot(sim_data.time, sim_data.imu_gyr(1, :), 'r-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.imu_gyr(2, :), 'g-', 'linewidth', 2.0);
%   plot(sim_data.time, sim_data.imu_gyr(3, :), 'b-', 'linewidth', 2.0);
%   xlabel("Time [s]");
%   ylabel("Angular Velocity [rad s^{-1}]");
%   legend('x', 'y', 'z');
%
%   ginput()
% end
