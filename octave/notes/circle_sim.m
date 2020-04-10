circle_r = 5.0;
nb_samples = 100;
% theta = -pi;
theta = 1.0;

sim_time = [];
x_pos = [];
y_pos = [];
x_vel = [];
y_vel = [];
x_acc = [];
y_acc = [];

dt = 0.001;
t_end = 10.0;

f = 1.0 / 10.0;
w = 2.0 * pi * f;

theta_k = theta;
theta_km1 = theta - (w * dt);
theta_km2 = theta_km1 - (w * dt);

ax = -circle_r * w**2 * cos(theta);
ay = -circle_r * w**2 * sin(theta);
vx = -circle_r * w * sin(theta);
vy = circle_r * w * cos(theta);
x = circle_r * cos(theta_k);
y = circle_r * sin(theta_k);

for t = 0.0:dt:t_end;
  sim_time = [sim_time; t];
  x_pos = [x_pos; x];
  y_pos = [y_pos; y];
  x_vel = [x_vel; vx];
  y_vel = [y_vel; vy];
  x_acc = [x_acc; ax];
  y_acc = [y_acc; ay];

  # Update theta
  theta += w * dt;

  # Update position
  x = x + vx * dt;
  y = y + vy * dt;

  # Update velocity
  vx = vx + ax * dt;
  vy = vy + ay * dt;

  # Update acceleration
  ax = -circle_r * w**2 * cos(theta);
  ay = -circle_r * w**2 * sin(theta);
endfor

figure(1);
plot(x_pos, y_pos);
xlabel("x [m]");
ylabel("y [m]");
% xlim([-circle_r, circle_r]);
% ylim([-circle_r, circle_r]);

figure(2);
subplot(311);
hold on;
plot(sim_time, x_pos, 'r-', 'linewidth', 2.0);
plot(sim_time, y_pos, 'g-', 'linewidth', 2.0);
xlabel("Time [s]");
ylabel("Displacement [m]");

subplot(312);
hold on;
plot(sim_time, x_vel, 'r-', 'linewidth', 2.0);
plot(sim_time, y_vel, 'g-', 'linewidth', 2.0);
xlabel("Time [s]");
ylabel("Velocity [ms^{-1}]");

subplot(313);
hold on;
plot(sim_time, x_acc, 'r-', 'linewidth', 2.0);
plot(sim_time, y_acc, 'g-', 'linewidth', 2.0);
xlabel("Time [s]");
ylabel("Acceleration [ms^{-2}]");

ginput();

% figure(1);
% hold on;
% theta = deg2rad(0:360);
% x_pos = circle_r * cos(theta);
% y_pos = circle_r * sin(theta);
% plot(x_pos, y_pos);
% ginput();
