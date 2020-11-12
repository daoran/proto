graphics_toolkit("fltk");
addpath(genpath("proto"));

C_WS = [
  -0.324045, 0.036747, -0.945328;
  0.036747, 0.998980, 0.026236;
  0.945328, -0.026236, -0.325065;
];

T_SC0 = [
	0.000000, -1.000000, 0.000000, 0.000000
	1.000000, 0.000000, 0.000000, 0.000000
	0.000000, 0.000000, 1.000000, 0.000000
	0.000000, 0.000000, 0.000000, 1.000000
];

T_C0F = [
  0.986976, -0.069762, 0.144956, -0.426290;
  -0.082567, -0.993015, 0.084277, 0.196928;
  0.138064, -0.095148, -0.985842, 0.959312;
  0.000000, 0.000000, 0.000000, 1.000000
];

r_C0F = tf_trans(T_C0F);
C_C0F = tf_rot(T_C0F);

T_WS = tf(C_WS, zeros(3, 1));
T_WF = T_WS * T_SC0 * T_C0F

% Calculate offset
offset = tf_trans(T_WF) * -1

% Set T_WF as origin and adjust T_WS accordingly
T_WF = tf(tf_rot(T_WF), zeros(3, 1))
T_WS = tf(tf_rot(T_WS), offset)


% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WS, 0.1);
draw_frame(T_WF, 0.1);

xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
