function [H_marg, g_marg] = schurs_complement(H, g, m, r, precond=false)
  assert(rows(H) == (m + r));

  % H = [Hmm, Hmr
  %      Hrm, Hrr];
  Hmm = H(1:m, 1:m);
  Hmr = H(1:m, m+1:m+r);
  Hrm = Hmr';
  Hrr = H(m+1:m+r, m+1:m+r);

  % g = [gmm, grr];
  gmm = g(1:m);
  grr = g(m+1:m+r);

  % Precondition Hmm
  if (precond)
    Hmm = 0.5 * (Hmm + Hmm');
  endif

  % Invert Hmm
  [V, lambda] = eig(Hmm);
  lambda_inv = diag(1.0 / diag(lambda));
  Hmm_inv = V * lambda_inv * V';

  % Schurs complement
  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  g_marg = grr - Hrm * Hmm_inv * gmm;
endfunction