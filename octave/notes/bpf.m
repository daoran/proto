addpath(genpath("prototype"));

% Body pose
r_WB = [0.0; 0; 0];
rpy_WB = [deg2rad(0.0); deg2rad(20.0); deg2rad(90.0)];
C_WB = euler321(rpy_WB);
T_WB = tf(C_WB, r_WB);

% Landing target pose
r_WZ = [1.0, 0.0, -1.0];
C_WZ = eye(3);
T_WZ = tf(C_WZ, r_WZ);

% Landing target to body frame
T_BW = inv(T_WB);
T_BZ = T_BW * T_WZ;
r_BZ = tf_trans(T_BZ);

% Body planar frame
rpy_PB = [deg2rad(0.0); deg2rad(20.0); deg2rad(0.0)];
C_PB = euler321(rpy_PB)

% Body planar frame
C_WB = tf_rot(T_WB);
rpy_WB = quat2euler(rot2quat(C_WB))
rpy_PB = [rpy_WB(1); rpy_WB(2); deg2rad(0.0)];
C_PB = euler321(rpy_PB)

r_PZ = C_PB * r_BZ

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WB, 1.0);
scatter3(r_WZ(1), r_WZ(2), r_WZ(3), "r", "linewidth", 2.0);
axis equal;
ginput();
