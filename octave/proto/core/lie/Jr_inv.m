function J = Jr_inv(theta)
  theta_norm = norm(theta);
  theta_skew = skew(theta);
  theta_skew_sq = theta_skew * theta_skew;

  A = 1.0 / theta_skew_sq;
  B = (1 + cos(theta_norm)) / (2 * theta_norm * sin(theta_norm));

  J = eye(3);
  J += 0.5 * theta_skew;
  J += (A - B) * theta_skew_sq;
endfunction
