function [H_marg, g_marg] = schurs_complement(H, g, m, r, precond=false)
  assert(rows(H) == (m + r));

  % H = [Hmm, Hmr
  %      Hrm, Hrr];
  Hmm = H(1:m, 1:m);
  Hmr = H(1:m, m+1:end);
  Hrm = Hmr';
  Hrr = H(m+1:end, m+1:end);

  % g = [gmm, grr];
  gmm = g(1:m);
  grr = g(m+1:m+r);

  % Precondition Hmm
  if (precond)
    Hmm = 0.5 * (Hmm + Hmm');
  endif

  % Invert Hmm
  assert(rank(Hmm) == rows(Hmm));
  [V, Lambda] = eig(Hmm);
  lambda = diag(Lambda);
  lambda_inv = 1.0 ./ lambda;
  Lambda_inv = diag(lambda_inv);
  Hmm_inv = V * Lambda_inv * V';

  % Schurs complement
  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  g_marg = grr - Hrm * Hmm_inv * gmm;
endfunction
