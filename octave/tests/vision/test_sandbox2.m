addpath(genpath("prototype"));

r_WS = [0.0; 0.0; 0.0];
R_WS = euler321(deg2rad([0.0; 0.0; 0.0]));
T_WS = tf(R_WS, r_WS)

figure();
hold on;
draw_frame(T_WS);
view(3);
xlabel("x");
ylabel("y");
zlabel("z");
ginput();
