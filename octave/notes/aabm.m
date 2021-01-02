% # Point relative to camera (HYPOTHETICAL)
% p_CPi = [0.0; 0.0; 0.5];
%
% # Marker-camera exintrinsics (REAL)
% q_MC = [0.160825; -0.683; 0.6954; -0.153];
% r_MC = [0.104008; 0.019301; -0.104969];
%
% # Marker world pose (HYPOTHETICAL)
% q_WM = [1.0; 0.0; 0.0; 0.0];
% r_WM = [0.0; 0.0; 0.0];
%
% # Back project
% # -- Parameters
% res = [640; 480];
% hfov = deg2rad(87);
% vfov = deg2rad(58);
% fx = 599.503798;
% fy = 601.315908;
% cx = 324.227980;
% cy = 243.189442;
% reproj_error = 1.5;  # (RMSE) Obtained from calibration
% # -- Measurements
% z = [320; 240];
% depth = 0.5;
% # -- Estimate uncertainty in world frame
% # Calculate conversion of pixel at a depth of 0.5m to meters
% pix2meters_horiz = (depth * tan(hfov / 2.0)) / (res(1) / 2);
% pix2meters_vert = (depth * tan(vfov / 2.0)) / (res(2) / 2);
% p_CPi_x_stddev = reproj_error * pix2meters_horiz
% p_CPi_y_stddev = reproj_error * pix2meters_vert
%
% # Note: OptiTrack claims accuracy is less than 0.05deg with 0.3mm errors
% # -- Form input covariance matrix
% covar_in = eye(15);
% covar_in(1:3, 1:3) = eye(3) * deg2rad(0.05)**2;  # theta_WM variance
% covar_in(4:6, 4:6) = eye(3) * 0.0003**2;         # r_WM variance
% covar_in(7, 7) = deg2rad(0.084)**2;              # theta_MC x variance
% covar_in(8, 8) = deg2rad(0.081)**2;              # theta_MC y variance
% covar_in(9, 9) = deg2rad(0.044)**2;              # theta_MC z variance
% covar_in(10, 10) = 0.001**2;                     # r_MC x variance
% covar_in(11, 11) = 0.001**2;                     # r_MC y variance
% covar_in(12, 12) = 0.001**2;                     # r_MC z variance
% covar_in(13, 13) = p_CPi_x_stddev**2;            # p_CPi variance
% covar_in(14, 14) = p_CPi_y_stddev**2;            # p_CPi variance
% covar_in(15, 15) = 0.00375**2;                   # p_CPi variance
%
% # If the Intel RealSense d435i camera is 1m from the object, the expected
% # accuracy is between 2.5mm to 5mm. Therefore a depth of 0.5m infers an
% # accuracy of 3.75mm.
% # https://www.intel.co.uk/content/www/uk/en/support/articles/000026260/emerging-technologies/intel-realsense-technology.html
%
%
% function R = quat2rot(q)
%   qw = q(1);
%   qx = q(2);
%   qy = q(3);
%   qz = q(4);
%
%   qx2 = qx**2;
%   qy2 = qy**2;
%   qz2 = qz**2;
%   qw2 = qw**2;
%
%   # Homogeneous form
%   R11 = qw2 + qx2 - qy2 - qz2;
%   R12 = 2 * (qx * qy - qw * qz);
%   R13 = 2 * (qx * qz + qw * qy);
%
%   R21 = 2 * (qx * qy + qw * qz);
%   R22 = qw2 - qx2 + qy2 - qz2;
%   R23 = 2 * (qy * qz - qw * qx);
%
%   R31 = 2 * (qx * qz - qw * qy);
%   R32 = 2 * (qy * qz + qw * qx);
%   R33 = qw2 - qx2 - qy2 + qz2;
%
%   R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
% endfunction;
%
% function p_prime = tf_point(T, p)
%   assert(length(p) == 3);
%   p_prime = (T * [p; 1.0])(1:3);
% endfunction
%
% function y = skew(x)
%   y = [0, -x(3), x(2);
%        x(3), 0, -x(1);
%        -x(2), x(1), 0];
% endfunction
%
% function R = rvec2rot(rvec)
%   % If small rotation
%   theta = sqrt(rvec(:)'*rvec(:));  % = norm(rvec), but faster
%   if theta < eps
%     R = [1, -rvec(3), rvec(2);
%           rvec(3), 1, -rvec(1);
%           -rvec(2), rvec(1), 1];
%     return
%   end
%
%   % Convert rvec to rotation matrix
%   rvec = rvec / theta;
%   x = rvec(1);
%   y = rvec(2);
%   z = rvec(3);
%
%   c = cos(theta);
%   s = sin(theta);
%   C = 1 - c;
%
%   xs = x * s;
%   ys = y * s;
%   zs = z * s;
%
%   xC = x * C;
%   yC = y * C;
%   zC = z * C;
%
%   xyC = x * yC;
%   yzC = y * zC;
%   zxC = z * xC;
%
%   R = [x * xC + c, xyC - zs, zxC + ys;
%        xyC + zs, y * yC + c, yzC - xs;
%        zxC - ys, yzC + xs, z * zC + c];
%   return
% endfunction
%
% function T = tf(rot, trans)
%   assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
%   assert(size(trans) == [3, 1]);
%
%   C = rot;
%   if size(rot) == [4, 1]
%     C = quat2rot(rot);
%   endif
%
%   T = eye(4, 4);
%   T(1:3, 1:3) = C;
%   T(1:3, 4) = trans;
% endfunction
%
% function C = tf_rot(tf)
%   C = tf(1:3, 1:3);
% endfunction
%
% function r = tf_trans(T)
%   r = T(1:3, 4);
% endfunction
%
% function [T_diff] = perturb_rot(T, step_size, i)
%   rvec = eye(3) * step_size;
%   C = tf_rot(T);
%   r = tf_trans(T);
%
%   C_diff = rvec2rot(rvec(1:3, i));
%   C_diff = C_diff * C;
%
%   T_diff = tf(C_diff, r);
% endfunction
%
% function [T_diff] = perturb_trans(T, step_size, i)
%   dr = eye(3) * step_size;
%   C = tf_rot(T);
%   r = tf_trans(T);
%   r_diff = r + dr(1:3, i);
%   T_diff = tf(C, r_diff);
% endfunction
%
% function retval = check_jacobian(jac_name, fdiff, jac, threshold, print=1)
%   % delta = sqrt(sum((fdiff - jac)(:))**2);
%   % if (delta > threshold)
%
%   d = (fdiff - jac);
%   failed = false;
%
%   for i = 1:rows(d)
%     for j = 1:columns(d)
%       delta = d(i, j);
%       if (abs(delta) > threshold)
%         failed = true;
%       endif
%     endfor
%   endfor
%
%   if failed
%     retval = -1;
%     if print
%       printf("Check [%s] failed!\n", jac_name);
%     endif
%     fdiff_minus_jac = fdiff - jac
%     num_diff = fdiff
%     jac
%     % delta
%     if print
%       printf("----------------------------------------\n");
%     endif
%   else
%     printf("Check [%s] passed!\n", jac_name);
%     % if print
%     %   printf("Check [%s] passed!\n", jac_name);
%     % endif
%     retval = 0;
%   endif
% endfunction
%
% % function check_ray_jac(J_ray, fx, fy, cx, cy, depth, z)
% %   step_size = 1e-6;
% %   threshold = 1e-4;
% %   z_descaled = [(z(1) - cx) / fx; (z(2) - cy) / fy];  # Descale-decenter z -> z'
% %   p_CPi = [z_descaled(1) * depth; z_descaled(2) * depth; depth];  # Back-project z' to 3D
% %
% %   step = eye(2) * step_size;
% %   fdiff = zeros(3, 2);
% %   for i = 1:2
% %     z_diff = z;
% %     z_diff(i) += step_size;
% %
% %     z_descaled_diff = [(z_diff(1) - cx) / fx; (z_diff(2) - cy) / fy];  # Descale-decenter z -> z'
% %     p_CPi_diff = [z_descaled_diff(1) * depth; z_descaled_diff(2) * depth; depth];  # Back-project z' to 3D
% %     fdiff(1:3, i) = (p_CPi_diff - p_CPi) / step_size;
% %   endfor
% %   fdiff
% %
% %   % retval = check_jacobian("J_ray", fdiff, J_ray, threshold);
% % endfunction
%
% function check_marker_pose_jac(J_WM, T_WM, T_MC, p_CPi)
%   step_size = 1e-6;
%   threshold = 1e-4;
%   p_WPi = tf_point(T_WM * T_MC, p_CPi);
%
%   step = eye(3) * step_size;
%   fdiff = zeros(3, 3);
%   for i = 1:3
%     T_WM_diff = perturb_rot(T_WM, step_size, i);
%     p_WPi_diff = tf_point(T_WM_diff * T_MC, p_CPi);
%     fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
%   endfor
%   retval = check_jacobian("J_theta_WM", fdiff, J_WM(1:3, 1:3), threshold);
%
%   step = eye(3) * step_size;
%   fdiff = zeros(3, 3);
%   for i = 1:3
%     T_WM_diff = perturb_trans(T_WM, step_size, i);
%     p_WPi_diff = tf_point(T_WM_diff * T_MC, p_CPi);
%     fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
%   endfor
%   retval = check_jacobian("J_r_WM", fdiff, J_WM(1:3, 4:6), threshold);
% endfunction
%
% function check_marker_camera_jac(J_MC, T_WM, T_MC, p_CPi)
%   step_size = 1e-8;
%   threshold = 1e-4;
%   p_WPi = tf_point(T_WM * T_MC, p_CPi);
%
%   step = eye(3) * step_size;
%   fdiff = zeros(3, 3);
%   for i = 1:3
%     T_MC_diff = perturb_rot(T_MC, step_size, i);
%     p_WPi_diff = tf_point(T_WM * T_MC_diff, p_CPi);
%     fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
%   endfor
%   retval = check_jacobian("J_theta_MC", fdiff, J_MC(1:3, 1:3), threshold);
%
%   step = eye(3) * step_size;
%   fdiff = zeros(3, 3);
%   for i = 1:3
%     T_MC_diff = perturb_trans(T_MC, step_size, i);
%     p_WPi_diff = tf_point(T_WM * T_MC_diff, p_CPi);
%     fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
%   endfor
%   retval = check_jacobian("J_r_MC", fdiff, J_MC(1:3, 4:6), threshold);
% endfunction
%
% function check_point_jac(J_p, T_WM, T_MC, p_CPi)
%   step_size = 1e-8;
%   threshold = 1e-4;
%   p_WPi = tf_point(T_WM * T_MC, p_CPi);
%
%   step = eye(3) * step_size;
%   fdiff = zeros(3, 3);
%   for i = 1:3
%     p_CPi_diff = p_CPi;
%     p_CPi_diff(i) += step_size;
%     p_WPi_diff = tf_point(T_WM * T_MC, p_CPi_diff);
%     fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
%   endfor
%   retval = check_jacobian("J_p", fdiff, J_p(1:3, 1:3), threshold);
% endfunction
%
% % # -- Measurement model
% % z_descaled = [(z(1) - cx) / fx; (z(2) - cy) / fy];  # Descale-decenter z -> z'
% % p_CPi = [z_descaled(1) * depth; z_descaled(2) * depth; depth];  # Back-project z' to 3D
% # -- Form back project jacobian
% % J_backproj = [depth, 0; 0, depth];
% % J_descale = [1 / fx, 0; 0, 1 / fy];
% % J_ray = J_backproj * J_descale;
% % check_ray_jac(J_ray, fx, fy, cx, cy, depth, z)
% % covar_calib = eye(2) * rmse_reproj_error**2;
% % covar_ray = J * covar_calib * J';
% % ray_var = diag(covar_ray)
% % ray_std = sqrt(ray_var)
%
% # Marker-Camera extrinsics
% T_MC = eye(4);
% T_MC(1:3, 1:3) = quat2rot(q_MC);
% T_MC(1:3, 4) = r_MC;
%
% # Marker world pose
% T_WM = eye(4);
% T_WM(1:3, 1:3) = quat2rot(q_WM);
% T_WM(1:3, 4) = r_WM;
%
% # Jacobians
% p_MPi = tf_point(T_MC, p_CPi);
% C_MC = T_MC(1:3, 1:3);
% C_WM = T_WM(1:3, 1:3);
% C_WC = (T_WM * T_MC)(1:3, 1:3);
% # -- Marker pose jacobian
% J_theta_WM = -skew(C_WM * p_MPi);
% J_r_WM = eye(3);
% J_WM = [J_theta_WM, J_r_WM];
% # -- Marker-camera jacobian
% J_theta_MC = C_WM * -skew(C_MC * p_CPi);
% J_r_MC = C_WM;
% J_MC = [J_theta_MC, J_r_MC];
% # -- Point jacobian
% J_p_CPi = C_WC;
% # -- Overall jacobian
% J = [J_theta_WM, J_r_WM, J_theta_MC, J_r_MC, J_p_CPi];
%
% # Check jacobians
% check_marker_pose_jac(J_WM, T_WM, T_MC, p_CPi);
% check_marker_camera_jac(J_MC, T_WM, T_MC, p_CPi);
% check_point_jac(J_p_CPi, T_WM, T_MC, p_CPi);
%
% # Calculate covariance of point in world frame: p_WPi
% # -- Calculate output covariance matrix using first order error propagation
% covar_out = J * covar_in * J';
% # -- p_WCi variance
% p_WCi_var = diag(covar_out);
% p_WCi_stddev = sqrt(p_WCi_var)


addpath(genpath("proto"));
graphics_toolkit("fltk");

euler_WM = deg2rad([0.0, 0.0, 0.0]);
q_WM = euler2quat(euler_WM);
r_WM = [0.0; 0.0; 0.0];
T_WM = tf(q_WM, r_WM);

% r_MC = [0.104008; 0.019301; -0.104969];
% q_MC = [0.160825; -0.683337; 0.695442; -0.153447]

r_MC = [0.099555; 0.015088; -0.091273];
q_MC = [-0.154034; 0.694426; -0.687994; 0.143910];

T_MC = tf(q_MC, r_MC);

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WM, 0.1);
draw_frame(T_WM * T_MC, 0.1);
xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
