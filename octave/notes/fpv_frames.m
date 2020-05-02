addpath(genpath("proto"));

euler_WS = deg2rad([180.0, 0.0, 0.0]);
q_WS = euler2quat(euler_WS);
r_WS = [0.0; 0.0; 0.1];
T_WS = tf(q_WS, r_WS);

euler_SC = deg2rad([90.0, 0.0, 90.0]);
q_SC = euler2quat(euler_SC);
r_SC = [0.1; 0.0; 0.0];
T_SC = tf(q_SC, r_SC);

T_SC

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WS, 0.1);
draw_frame(T_WS * T_SC, 0.1);
xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
