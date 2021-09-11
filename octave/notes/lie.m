addpath(genpath("proto"));

# Camera pose T_WC0
rot = euler321(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC0 = tf(rot, trans);
C_WC0 = tf_rot(T_WC0);

# Perturb using exponential map
perturb = deg2rad([0.01; 0.2; 0.2])
C_original = C_WC0
C_perturbed = C_WC0 * Exp(perturb)

# Find perturbation using logarithmic map
% C_perturbed - C_original
perturb_recover = Log(inv(C_original) * C_perturbed)



