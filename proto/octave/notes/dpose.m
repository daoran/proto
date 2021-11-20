addpath(genpath("proto"));
graphics_toolkit("fltk");

# Pose i
rpy_i = deg2rad([0.0, 0.0, 0.0]);
C_i = euler321(rpy_i);
r_i = [0.0; 0.0; 0.0];
T_i = tf(C_i, r_i);

# Pose j
rpy_j = deg2rad([0.0, 0.0, 90.0]);  % z is perturbed by 0.1 rad
C_j = euler321(rpy_j);
r_j = [0.0; 0.0; 0.0];
T_j = tf(C_j, r_j);

# Difference between pose i and j
dT = T_j * inv(T_i)

# Extract rotation and convert to quatnernion
dC = dT(1:3, 1:3)
dquat = rot2quat(dC)

# Now convert difference in quaternion to small perturbations in x, y, z
dtheta = 2 * dquat(2:4)  % should show that z is perturbed by around 0.01 rad
