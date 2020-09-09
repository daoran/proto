addpath(genpath("proto"));
graphics_toolkit("fltk");

# Camera pose
r_WC = [0.0; 0.0; 0.0];
rpy_WC = deg2rad([-90.0, 0.0, -90.0]);
C_WC = euler321(rpy_WC);
T_WC = tf(C_WC, r_WC);

# 3D point in world frame
p_C = [0.0; 0.0; 3.0];
hp_C = homogeneous(p_C);
p_W = dehomogeneous(T_WC * hp_C);

# Camera intrinsics
image_width = 160;
image_height = 120;
hfov = 90;
vfov = 60;
fx = focal_length(image_width, hfov);
fy = focal_length(image_height, vfov);
cx = image_width / 2.0;
cy = image_height / 2.0;
K = pinhole_K([fx, fy, cx, cy]);

# Hypothetical measurement covariance
stddev_fx = 1.0;
stddev_fy = 1.0;
stddev_cx = 1.0;
stddev_cy = 1.0;
var_fx = stddev_fx**2;
var_fy = stddev_fy**2;
var_cx = stddev_cx**2;
var_cy = stddev_cy**2;
covar_params = diag([var_fx, var_fy, var_cx, var_cy])
% covar_params = rand(4, 4) + diag([1.0, 1.0, 1.0, 1.0])

% Plot uncertainty map
function p_C = back_project(K, pixel)
  p_C = inv(K) * homogeneous(pixel);
endfunction

function J = dz__dparams(p_C)
  J = [p_C(1) / p_C(3), 0.0, 1.0, 0.0;
       0.0, p_C(2) / p_C(3), 0.0, 1.0];
endfunction

entropy_map = zeros(image_height, image_width);
for y = 1:image_height
  printf(".");
  for x = 1:image_width
    p_C = back_project(K, [x; y]);
    J = dz__dparams(p_C);
    covar_point = J * covar_params * J';
    J
    covar_point
    exit

    n = length(covar_point);
    entropy = 0.5 * log((2 * pi * e)^n * det(covar_point));
    entropy_map(y, x) = entropy;
  endfor
endfor

% min(min(entropy_map))
% max(max(entropy_map))

figure();
imagesc(entropy_map)
title("Uncertainty map of camera intrinsics [fx, fy, cx, cy]");
xlabel("x [px]");
ylabel("y [px]");
xlim([0, image_width]);
ylim([0, image_height]);
axis("square");
colorbar();
ginput();


% # Plot
% figure(1);
% hold on;
% draw_camera(T_WC);
% draw_frame(T_WC, 0.1);
%
% scatter3(p_W(1), p_W(2), p_W(3), 10.0, "b", "filled");
% title("Error Propagation example");
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% view(3);
% axis 'equal';
%
% ginput();
