addpath(genpath("proto"));
graphics_toolkit("fltk");

calib_target = calib_target_init();
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [0.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);
target_pos = tf_trans(T_WT);
cam_pos = [-0.5; 0.1; 0];

% up_axis = [0; 1; 0];
% cam_f = normalize((target_pos - cam_pos))
% cam_l = normalize(cross(normalize(up_axis), cam_f))
% cam_u = normalize(cross(cam_f, cam_l))
%
% T_TC = [cam_f(1), cam_f(2), cam_f(3), cam_pos(1);
%         cam_l(1), cam_l(2), cam_l(3), cam_pos(2);
%         cam_u(1), cam_u(2), cam_u(3), cam_pos(3);
%         0, 0, 0, 1;]
% % T_WC = T_WT * T_TC;
% T_WC = T_TC;
%
% % T_WC = [0, 0, 1, cam_pos(1);
% %         0, 1, 0, cam_pos(2);
% %         1, 0, 0, cam_pos(3);
% %         0, 0, 0, 1;]
%
% % T_WC = [0, 1, 0, cam_pos(1);
% %         1, 0, 0, cam_pos(2);
% %         0, 0, 1, cam_pos(3);
% %         0, 0, 0, 1;]
%
% det(tf_rot(T_WC))

% T_TC = lookat(cam_pos, target_pos, up_axis)
T_TC = lookat(cam_pos, target_pos)
T_WC = T_WT * T_TC;
% T_WC = T_TC;

% T_WC = [1, 0, 0, cam_pos(1);
%         0, 1, 0, cam_pos(2);
%         0, 0, 1, cam_pos(3);
%         0, 0, 0, 1;];

figure(1);
hold on;
draw_frame(T_WC, 0.1);
scatter3(0, 0, 0, 50, 'filled');
calib_target_draw(calib_target, T_WT);

padding = 1.0;
xlim([min(target_pos(1), cam_pos(1)) - padding, max(target_pos(1), cam_pos(1)) + padding]);
ylim([min(target_pos(2), cam_pos(2)) - padding, max(target_pos(2), cam_pos(2)) + padding]);
xlabel("x");
ylabel("y");
zlabel("z");
axis("equal");
view(3);
ginput();
