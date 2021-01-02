addpath(genpath("."));
graphics_toolkit("fltk");

# Settings
step_size = 1.0e-9;
threshold = 1.0e-5;


r_FFi = [0.1; 0.1; 0.0];
C_WF = euler321(deg2rad([90.0; 0.0; -90.0]));
r_WF = zeros(3, 1);
T_WF = tf(C_WF, r_WF);

C_C0F = euler321(deg2rad([180.0; 0.0; 0.0]));
r_C0F = [0.0; 1.0; 1.0];
T_C0F = tf(C_C0F, r_C0F);

r_C1C0 = [0.2; 0.0; 0.0];
% C_C1C0 = eye(3);
C_C1C0 = euler321(deg2rad([0.1; 0.2; 0.3]));
T_C1C0 = tf(C_C1C0, r_C1C0);

T_C1F = T_C1C0 * T_C0F;

% r_C0Fi = tf_point(T_C0F, r_FFi)
% r_C0Fi = tf_point(T_C0F, r_FFi)

function r_C0Fi = h_cam0(T_C1C0, T_C1F, r_FFi)
  r_C0Fi = tf_point(inv(T_C1C0) * T_C1F, r_FFi);
endfunction

function r_C1Fi = h_cam1(T_C1C0, T_C0F, r_FFi)
  r_C1Fi = tf_point(T_C1C0 * T_C0F, r_FFi);
endfunction

function retval = check_jac_wrt_r_C0F(T_C1C0, T_C0F, T_C1F, r_FFi, jac, step_size, threshold)
  fdiff = zeros(6, 3);
  C_C0F = tf_rot(T_C0F);
  r_C0F = tf_trans(T_C0F);
  r_C1F = tf_trans(T_C1F);
  r_C0Fi = tf_point(T_C0F, r_FFi);
  r_C1Fi = tf_point(T_C1F, r_FFi);

  r = h_cam0(T_C1C0, T_C1F, r_FFi);
  for i = 1:3
    r_C0F_diff = perturb_trans(r_C0F, step_size, i);
    T_C0F_diff = tf(C_C0F, r_C0F_diff);
    r_prime = h_cam0(T_C0F_diff, T_C1F, r_FFi);
    fdiff(1:3, i) = (r_prime - r) / step_size;
  endfor

  r = h_cam1(T_C1C0, T_C0F, r_FFi);
  for i = 1:3
    r_C0F_diff = perturb_trans(r_C0F, step_size, i);
    T_C0F_diff = tf(C_C0F, r_C0F_diff);
    r_prime = h_cam1(T_C0F_diff, T_C0F, r_FFi);
    fdiff(4:6, i) = (r_prime - r) / step_size;
  endfor

  fdiff
  retval = check_jacobian("J_r_C0F", fdiff, jac, threshold);
endfunction

function retval = check_jac_wrt_C_C1C0(T_C1C0, T_C0F, T_C1F, r_FFi, jac, step_size, threshold)
  fdiff = zeros(6, 3);
  r_C1C0 = tf_trans(T_C1C0);
  C_C1C0 = tf_rot(T_C1C0);
  r_C0Fi = tf_point(T_C0F, r_FFi);
  r_C1Fi = tf_point(T_C1F, r_FFi);

  r = h_cam0(T_C1C0, T_C1F, r_FFi);
  for i = 1:3
    C_C1C0_diff = perturb_rot(C_C1C0, step_size, i);
    T_C1C0_diff = tf(C_C1C0_diff, r_C1C0);
    r_prime = h_cam0(T_C1C0_diff, T_C1F, r_FFi);
    fdiff(1:3, i) = (r_prime - r) / step_size;
  endfor

  r = h_cam1(T_C1C0, T_C0F, r_FFi);
  for i = 1:3
    C_C1C0_diff = perturb_rot(C_C1C0, step_size, i);
    T_C1C0_diff = tf(C_C1C0_diff, r_C1C0);
    r_prime = h_cam1(T_C1C0_diff, T_C0F, r_FFi);
    fdiff(4:6, i) = (r_prime - r) / step_size;
  endfor

  retval = check_jacobian("J_C_C1C0", fdiff, jac, threshold);
endfunction

function retval = check_jac_wrt_r_C1C0(T_C1C0, T_C0F, T_C1F, r_FFi, jac, step_size, threshold)
  fdiff = zeros(6, 3);
  r_C1C0 = tf_trans(T_C1C0);
  C_C1C0 = tf_rot(T_C1C0);
  r_C0Fi = tf_point(T_C0F, r_FFi);
  r_C1Fi = tf_point(T_C1F, r_FFi);

  r = h_cam0(T_C1C0, T_C1F, r_FFi);
  for i = 1:3
    r_C1C0_diff = perturb_trans(r_C1C0, step_size, i);
    T_C1C0_diff = tf(C_C1C0, r_C1C0_diff);
    r_prime = h_cam0(T_C1C0_diff, T_C1F, r_FFi);
    fdiff(1:3, i) = (r_prime - r) / step_size;
  endfor

  r = h_cam1(T_C1C0, T_C0F, r_FFi);
  for i = 1:3
    r_C1C0_diff = perturb_trans(r_C1C0, step_size, i);
    T_C1C0_diff = tf(C_C1C0, r_C1C0_diff);
    r_prime = h_cam1(T_C1C0_diff, T_C0F, r_FFi);
    fdiff(4:6, i) = (r_prime - r) / step_size;
  endfor

  fdiff
  retval = check_jacobian("J_r_C1C0", fdiff, jac, threshold);
endfunction

r_C0Fi = tf_point(T_C0F, r_FFi);
r_C1Fi = tf_point(T_C1F, r_FFi);


% Jacobians w.r.t. T_C0F
% J_r_C0F = [-skew(C_C0F * r_FFi); eye(3)];
J_r_C0F = [eye(3); eye(3)];
check_jac_wrt_r_C0F(T_C1C0, T_C0F, T_C1F, r_FFi, J_r_C0F, step_size, threshold)


% Jacobians w.r.t. T_C1C0
J_r_C1C0 = [-C_C1C0';
            eye(3)];
J_C_C1C0 = [-skew(C_C1C0' * (r_C1Fi - r_C1C0)) * -C_C1C0';
            -skew(C_C1C0 * r_C0Fi)];

check_jac_wrt_r_C1C0(T_C1C0, T_C0F, T_C1F, r_FFi, J_r_C1C0, step_size, threshold)
check_jac_wrt_C_C1C0(T_C1C0, T_C0F, T_C1F, r_FFi, J_C_C1C0, step_size, threshold)


% % Visualize
% figure(1);
% hold on;
% draw_frame(T_WF * inv(T_C0F), 0.3);
% draw_frame(T_WF * inv(T_C0F) * inv(T_C1C0), 0.3);
% draw_frame(T_WF, 0.3);
%
% xlabel("x");
% ylabel("y");
% zlabel("z");
% axis("equal");
% view(3);
% ginput();
