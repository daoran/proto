addpath(genpath("prototype"));

% Initialize bicycle model
x = 0.0;
y = 0.0;
theta = 0.0;
l = 0.5;
bicycle = bicycle_init(x, y, theta, l);

% Simulation settings
t = 0.0;
t_end = 10.0;
dt = 0.01;
v = 10.0;  % bicycle velocity
delta = deg2rad(20.0);  % bicycle steering angle

% Simulate
state_history = [bicycle.state];
for i = 0:dt:t_end
  bicycle = bicycle_update(bicycle, v, delta, dt);
  state_history = [state_history, bicycle.state];
  t += dt;
endfor

% Visualize
debug = false;
if debug == true
  figure(1);
  plot(state_history(1, :), state_history(2, :), "r-");
  ginput();
endif
