addpath(genpath("proto"));
graphics_toolkit("fltk");

# Ri
rpy_i = deg2rad([0.0, 0.0, 0.0]);
R_i = euler321(rpy_i)

# Rj
rpy_j = deg2rad([0.0, 0.0, 20.0]);  % z is perturbed by 0.1 rad
R_j = euler321(rpy_j)

# Difference between pose i and j
dR = R_j * inv(R_i)

# Extract rotation and convert to quatnernion
dquat = rot2quat(dR)

# Now convert difference in quaternion to small perturbations in x, y, z
dtheta = 2 * dquat(2:4); % should show that z is perturbed by around 0.01 rad
dtheta = rad2deg(dtheta)
