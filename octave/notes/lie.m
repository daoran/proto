addpath(genpath("proto"));

% Camera pose T_WC0
rot = euler321(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC0 = tf(rot, trans);
C_WC0 = tf_rot(T_WC0);

# Perturb using exponential map
perturb = deg2rad([0.01; 0.2; 0.2]);
C_original = C_WC0;
C_perturbed = C_WC0 * Exp(perturb);

% Find perturbation using logarithmic map
% C_perturbed - C_original
perturb_recover = Log(inv(C_original) * C_perturbed);



% -------------------------- Useful Derivatives --------------------------------
step_size = 1e-8;
threshold = 1e-4;

% Derivative of the inverse of an orientation
% d(inv(rot)) / d(rot)

C_WC0 = euler321(deg2rad([-90; 0; -90]));

J_rot = -C_WC0;

perturb = eye(3) * step_size;
J_numdiff = zeros(3, 3);

for i = 1:3
  % Perturb original rotation
  C_prime = boxplus(C_WC0, perturb(1:3, i));

  % Calculate numerical diff
  J_numdiff(1:3, i) = boxminus(inv(C_prime), inv(C_WC0)) / step_size;
endfor

assert(check_jacobian("J_invrot", J_numdiff, J_rot, threshold, true) == 0);


% Derivative of coordinate map
% d(rot * p) / d(rot)

C_WC0 = euler321(deg2rad([-90; 0; -90]));
p_C0 = [0.1; 0.2; 0.3];
p_W = C_WC0 * p_C0;

J_rot = -C_WC0 * skew(p_C0);

perturb = eye(3) * step_size;
J_numdiff = zeros(3, 3);

for i = 1:3
  % Perturb original rotation
  C_WC0_prime = boxplus(C_WC0, perturb(1:3, i));
  p_W_prime = C_WC0_prime * p_C0;

  % Calculate numerical diff
  J_numdiff(1:3, i) = (p_W_prime - p_W) / step_size;
endfor

assert(check_jacobian("J_rot", J_numdiff, J_rot, threshold, true) == 0);


% Derivative of left concatenation
% d(C1 * C2) / d(C1)

C1 = euler321(deg2rad([10; 20; 30]));
C2 = euler321(deg2rad([40; 50; 60]));
C = C1 * C2;

J_lrot = C2';

perturb = eye(3) * step_size;
J_numdiff = zeros(3, 3);

for i = 1:3
  % Perturb original rotation
  C1_prime = boxplus(C1, perturb(1:3, i));
  C_prime = C1_prime * C2;

  % Calculate numerical diff
  J_numdiff(1:3, i) = boxminus(C_prime, C) / step_size;
endfor

assert(check_jacobian("J_lrot", J_numdiff, J_lrot, threshold, true) == 0);


% Derivative of right concatenation
% d(C1 * C2) / d(C2)

C1 = euler321(deg2rad([10; 20; 30]));
C2 = euler321(deg2rad([40; 50; 60]));
C = C1 * C2;

J_rrot = eye(3);

step_size = 1e-8;
threshold = 1e-4;
perturb = eye(3) * step_size;
J_numdiff = zeros(3, 3);

for i = 1:3
  % Perturb original rotation
  C2_prime = boxplus(C2, perturb(1:3, i));
  C_prime = C1 * C2_prime;

  % Calculate numerical diff
  J_numdiff(1:3, i) = boxminus(C_prime, C) / step_size;
endfor

assert(check_jacobian("J_rrot", J_numdiff, J_rrot, threshold, true) == 0);



% a = [1;2;3];
% b = [10;5;1];
% skew(a) * b
% -skew(b) * a
% a' * skew(b)
% -b' * skew(a)
