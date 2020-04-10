circle_r = 5.0;
nb_samples = 100;
theta = -pi;

sim_time = [];
x_pos = [];
y_pos = [];
x_vel = [];
y_vel = [];
x_acc = [];
y_acc = [];

dt = 0.1;
t_end = 10.0;

f = 1.0 / 10.0;
w = 2.0 * pi * f;

x = circle_r * cos(theta);
y = circle_r * sin(theta);
vx = -circle_r * sin(theta) * w;
vy = circle_r * cos(theta) * w;

for t = 0.0:dt:t_end;
  sim_time = [sim_time; t];
  x_pos = [x_pos; x];
  y_pos = [y_pos; y];
  x_vel = [x_vel; vx];
  y_vel = [y_vel; vy];

  # Update theta
  theta += w * dt;

  # Update velocity
  vx = -circle_r * sin(theta) * w;
  vy = circle_r * cos(theta) * w;

  # Update position
  x = x + vx * dt;
  y = y + vy * dt;
endfor

figure(1);
subplot(211);
hold on;
plot(x_pos, y_pos);
axis("equal");

subplot(212);
hold on;
plot(sim_time, x_vel, 'r-', 'linewidth', 2.0);
plot(sim_time, y_vel, 'g-', 'linewidth', 2.0);
ginput();

% figure(1);
% hold on;
% theta = deg2rad(0:360);
% x_pos = circle_r * cos(theta);
% y_pos = circle_r * sin(theta);
% plot(x_pos, y_pos);
% ginput();
