# Point relative to camera (HYPOTHETICAL)
p_CPi = [0.0; 0.0; 10.0];

# Marker-camera exintrinsics (REAL)
q_MC = [0.160825; -0.683; 0.6954; -0.153];
r_MC = [0.104008; 0.019301; -0.104969];

# Marker world pose (HYPOTHETICAL)
q_WM = [1.0; 0.0; 0.0; 0.0];
r_WM = [1.0; 2.0; 3.0];

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  # Homogeneous form
  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction;

function p_prime = tf_point(T, p)
  assert(length(p) == 3);
  p_prime = (T * [p; 1.0])(1:3);
endfunction

function y = skew(x)
  y = [0, -x(3), x(2);
       x(3), 0, -x(1);
       -x(2), x(1), 0];
endfunction

function R = rvec2rot(rvec)
  % If small rotation
  theta = sqrt(rvec(:)'*rvec(:));  % = norm(rvec), but faster
  if theta < eps
    R = [1, -rvec(3), rvec(2);
          rvec(3), 1, -rvec(1);
          -rvec(2), rvec(1), 1];
    return
  end

  % Convert rvec to rotation matrix
  rvec = rvec / theta;
  x = rvec(1);
  y = rvec(2);
  z = rvec(3);

  c = cos(theta);
  s = sin(theta);
  C = 1 - c;

  xs = x * s;
  ys = y * s;
  zs = z * s;

  xC = x * C;
  yC = y * C;
  zC = z * C;

  xyC = x * yC;
  yzC = y * zC;
  zxC = z * xC;

  R = [x * xC + c, xyC - zs, zxC + ys;
       xyC + zs, y * yC + c, yzC - xs;
       zxC - ys, yzC + xs, z * zC + c];
  return
endfunction

function T = tf(rot, trans)
  assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
  assert(size(trans) == [3, 1]);

  C = rot;
  if size(rot) == [4, 1]
    C = quat2rot(rot);
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = trans;
endfunction

function C = tf_rot(tf)
  C = tf(1:3, 1:3);
endfunction

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function [T_diff] = perturb_rot(T, step_size, i)
  rvec = eye(3) * step_size;
  C = tf_rot(T);
  r = tf_trans(T);

  C_diff = rvec2rot(rvec(1:3, i));
  C_diff = C_diff * C;

  T_diff = tf(C_diff, r);
endfunction

function [T_diff] = perturb_trans(T, step_size, i)
  dr = eye(3) * step_size;
  C = tf_rot(T);
  r = tf_trans(T);
  r_diff = r + dr(1:3, i);
  T_diff = tf(C, r_diff);
endfunction

function retval = check_jacobian(jac_name, fdiff, jac, threshold, print=true)
  % delta = sqrt(sum((fdiff - jac)(:))**2);
  % if (delta > threshold)

  d = (fdiff - jac);
  failed = false;

  for i = 1:rows(d)
    for j = 1:columns(d)
      delta = d(i, j);
      if (abs(delta) > threshold)
        failed = true;
      endif
    endfor
  endfor

  if failed
    retval = -1;
    if print
      printf("Check [%s] failed!\n", jac_name);
    endif
    fdiff_minus_jac = fdiff - jac
    num_diff = fdiff
    jac
    % delta
    if print
      printf("----------------------------------------\n");
    endif
  else
    printf("Check [%s] passed!\n", jac_name);
    % if print
    %   printf("Check [%s] passed!\n", jac_name);
    % endif
    retval = 0;
  endif
endfunction

function check_marker_pose_jac(J_WM, T_WM, T_MC, p_CPi)
  step_size = 1e-6;
  threshold = 1e-4;
  p_WPi = tf_point(T_WM * T_MC, p_CPi);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_WM_diff = perturb_rot(T_WM, step_size, i);
    p_WPi_diff = tf_point(T_WM_diff * T_MC, p_CPi);
    fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
  endfor
  retval = check_jacobian("J_theta_WM", fdiff, J_WM(1:3, 1:3), threshold);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_WM_diff = perturb_trans(T_WM, step_size, i);
    p_WPi_diff = tf_point(T_WM_diff * T_MC, p_CPi);
    fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
  endfor
  retval = check_jacobian("J_r_WM", fdiff, J_WM(1:3, 4:6), threshold);
endfunction

function check_marker_camera_jac(J_MC, T_WM, T_MC, p_CPi)
  step_size = 1e-8;
  threshold = 1e-4;
  p_WPi = tf_point(T_WM * T_MC, p_CPi);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_MC_diff = perturb_rot(T_MC, step_size, i);
    p_WPi_diff = tf_point(T_WM * T_MC_diff, p_CPi);
    fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
  endfor
  retval = check_jacobian("J_theta_MC", fdiff, J_MC(1:3, 1:3), threshold);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    T_MC_diff = perturb_trans(T_MC, step_size, i);
    p_WPi_diff = tf_point(T_WM * T_MC_diff, p_CPi);
    fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
  endfor
  retval = check_jacobian("J_r_MC", fdiff, J_MC(1:3, 4:6), threshold);
endfunction

function check_point_jac(J_p, T_WM, T_MC, p_CPi)
  step_size = 1e-8;
  threshold = 1e-4;
  p_WPi = tf_point(T_WM * T_MC, p_CPi);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    p_CPi_diff = p_CPi;
    p_CPi_diff(i) += step_size;
    p_WPi_diff = tf_point(T_WM * T_MC, p_CPi_diff);
    fdiff(1:3, i) = (p_WPi_diff - p_WPi) / step_size;
  endfor
  retval = check_jacobian("J_p", fdiff, J_p(1:3, 1:3), threshold);
endfunction

# Marker-Camera extrinsics
T_MC = eye(4);
T_MC(1:3, 1:3) = quat2rot(q_MC);
T_MC(1:3, 4) = r_MC;

# Marker world pose
T_WM = eye(4);
T_WM(1:3, 1:3) = quat2rot(q_WM);
T_WM(1:3, 4) = r_WM;

# Jacobians
p_MPi = tf_point(T_MC, p_CPi);
C_MC = T_MC(1:3, 1:3);
C_WM = T_WM(1:3, 1:3);
C_WC = (T_WM * T_MC)(1:3, 1:3);
# -- Marker pose jacobian
J_theta_WM = -skew(C_WM * p_MPi);
J_r_WM = eye(3);
J_WM = [J_theta_WM, J_r_WM];
# -- Marker-camera jacobian
J_theta_MC = C_WM * -skew(C_MC * p_CPi);
J_r_MC = C_WM;
J_MC = [J_theta_MC, J_r_MC];
# -- Point jacobian
J_p_CPi = C_WC;
# -- Overall jacobian
J = [J_theta_WM, J_r_WM, J_theta_MC, J_r_MC, J_p_CPi]

# Check jacobians
check_marker_pose_jac(J_WM, T_WM, T_MC, p_CPi);
check_marker_camera_jac(J_MC, T_WM, T_MC, p_CPi);
check_point_jac(J_p_CPi, T_WM, T_MC, p_CPi);

# Calculate covariance of point in world frame: p_WPi
# -- Form input covariance matrix
covar_in = eye(15);
covar_in(1:3, 1:3) = eye(3) * deg2rad(0.05)**2;  # theta_WM variance
covar_in(4:6, 4:6) = eye(3) * 0.003**2;          # r_WM variance
covar_in(7:9, 7:9) = eye(3) * deg2rad(0.5)**2;   # theta_MC variance
covar_in(10:12, 10:12) = eye(3) * 0.001**2;      # r_MC variance
covar_in(13:15, 13:15) = eye(3) * 0.001**2;      # p_CPi variance
# -- Calculate output covariance matrix using first order error propagation
covar_out = J * covar_in * J';
# -- p_WCi variance
diag(covar_out)
