% addpath(genpath("prototype"));
%
% image_width = 640;
% image_height = 480;
% fov = 90.0;
%
% r_WP = [randf([-1.0, 1.0]); randf([-1.0, 1.0]); 0.5];
% fx = focal_length(image_width, fov);
% fy = focal_length(image_height, fov);
% cx = image_width / 2.0;
% cy = image_height / 2.0;
% K = pinhole_K([fx, fy, cx, cy]);
%
% r_WC = [-3.0; 0.0; 1.0];
% R_WC = euler321(deg2rad([-90.0; 0.0; -90.0]));
% T_WC = tf(R_WC, r_WC);
%
% T_CW = inv(T_WC);
% r_CP = dehomogeneous(T_CW * homogeneous(r_WP));
%
% X = [r_CP(1) / r_CP(3); r_CP(2) / r_CP(3); 1.0];
% x = K * X;
%
% r_CP = inv(K) * x
% r_WP_est = dehomogeneous(T_WC * homogeneous(r_CP))
%
% p0 = r_WC
% p1 = r_WP
% n = [0; 0; 1];
% v = (p1 - p0);
%
% a = n(1);
% b = n(2);
% c = n(3);
% % p1 = [1.0, 0.0, 0.0];
% % d = (a * p1(1) + b * p1(2) + c * p1(3))
% d = 0
%
% t = -(p0' * n + d) / (v' * n);
% P = p0 + t * v;
% P
%
% figure();
% hold on;
% draw_frame(T_WC);
% plot3(r_WP(1), r_WP(2), r_WP(3), "ro");
% plot3(r_WP_est(1), r_WP_est(2), r_WP_est(3), "bo");
% plot3(P(1), P(2), P(3), "bo");
% draw_camera(T_WC);
% view(3);
% xlim([-5.0, 5.0]);
% ylim([-5.0, 5.0]);
% zlim([0.0, 5.0]);
% xlabel("x");
% ylabel("y");
% zlabel("z");
% ginput();


addpath(genpath("prototype"));

r_WC = [0.0; 5.0; 30.0];
R_WC = euler321(deg2rad([180.0; 0.0; 0.0]));
T_WC = tf(R_WC, r_WC)

r_CP = [-0.1; 0.1; 1.0];
hr_CP = homogeneous(r_CP);
hr_WC = T_WC * hr_CP;
r_WC = dehomogeneous(hr_WC)


figure();
hold on;
draw_frame(T_WC);
draw_camera(T_WC);
view(3);
xlabel("x");
ylabel("y");
zlabel("z");
ginput();
