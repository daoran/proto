addpath(genpath("proto"));

function derive_jacobians()
  pkg load symbolic;

  syms p_Fx p_Fy p_Fz;
  syms psi;
  syms theta;
  syms phi;

  p_F = [p_Fx; p_Fy; p_Fz];


  # Rotation about the z-axis
  Rz = [cos(psi), -sin(psi), sym(0);
        sin(psi), cos(psi), sym(0);
        sym(0), sym(0), sym(1)];

  # Rotation about the y-axis
  Ry = [cos(theta), sym(0), sin(theta);
        sym(0), sym(1), sym(0);
        -sin(theta), sym(0), cos(theta)];

  # Rotation about the x-axis
  Rx = [sym(1), sym(0), sym(0);
        sym(0), cos(phi), -sin(phi);
        sym(0), sin(phi), cos(phi)];

  J_phi = diff(Rz * Ry * Rx, phi)
  J_theta = diff(Rz * Ry * Rx, theta)
  J_psi = diff(Rz * Ry * Rx, psi)

  ccode(J_phi)
  ccode(J_theta)
  ccode(J_psi)
endfunction

% derive_jacobians()

# Point in fidicual frame
p_F = [0.1; 0.1; 0.0];

# Fiducial pose
r_WF = rand(3, 1);
euler = rand(3, 1);
C_WF = euler321(euler);
T_WF = tf(C_WF, r_WF);

# Transform point in fiducial frame to world frame
hp_W = T_WF * [p_F; 1.0];
p_W = hp_W(1:3);

# Form fiducial jacobians
phi = euler(1);
theta = euler(2);
psi = euler(3);

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);

J_x = [p_F(2) * (sphi * spsi + stheta * cphi * cpsi) + p_F(3) * (-sphi * stheta * cpsi + spsi * cphi),
       p_F(2) * (-sphi * cpsi + spsi * stheta * cphi) + p_F(3) * (-sphi * spsi * stheta - cphi * cpsi),
       p_F(2) * cphi * ctheta - p_F(3) * sphi * ctheta];
J_y = [-p_F(1) * stheta * cpsi + p_F(2) * sphi * cpsi * ctheta + p_F(3) * cphi * cpsi * ctheta;
       -p_F(1) * spsi * stheta + p_F(2) * sphi * spsi * ctheta + p_F(3) * spsi * cphi * ctheta;
       -p_F(1) * ctheta - p_F(2) * sphi * stheta - p_F(3) * stheta * cphi];
J_z = [-p_F(1) * spsi * ctheta + p_F(2) * (-sphi * spsi * stheta - cphi * cpsi) + p_F(3) * (sphi * cpsi - spsi * stheta * cphi);
       p_F(1) * cpsi * ctheta + p_F(2) * (sphi * stheta * cpsi - spsi * cphi) + p_F(3) * (sphi * spsi + stheta * cphi * cpsi);
       0];
J = [J_x, J_y, J_z];

# Numerical diff
step = 1e-6;
num_diff = zeros(3, 3);
for i = 1:3
  euler_fd = euler;
  euler_fd(i) += step;

  C_WF_fd = euler321(euler_fd);
  T_WF_fd = tf(C_WF_fd, r_WF);
  hp_W_fd = T_WF_fd * [p_F; 1.0];
  p_W_fd = hp_W_fd(1:3);

  num_diff(:, i) = (p_W_fd - p_W) / step;
end

# Check jacobians
if norm(J - num_diff) < 1e-6
  printf("Fiducial Jacobians are GOOD!\n");
else
  printf("Fiducial Jacobians are BAD!\n");
end
