function [H_marg, g_marg] = schurs_complement(H, g, m, r)
  % H = [Hmm, Hmr
  %      Hrm, Hrr];
  Hmm = H(1:m, 1:m);
  Hmr = H(1:m, m+1:m+r);
  Hrm = Hmr';
  Hrr = H(m+1:m+r, m+1:m+r);

  % g = [gmm, grr];
  gmm = g(1:m);
  grr = g(m+1:m+r);

  % Hmm = 0.5 * (Hmm + Hmm');

  % Invert Hmm
  [V, lambda] = eig(Hmm);
  lambda_inv = diag(1.0 / diag(lambda));
  Hmm_inv = V * lambda_inv * V';
  % Hmm_inv = pinv(Hmm);

  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  g_marg = grr - Hrm * Hmm_inv * gmm;

  % H_marg = zeros(2, 2);
  % g_marg = zeros(2, 2);
endfunction
