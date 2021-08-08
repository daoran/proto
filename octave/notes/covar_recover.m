% Recover a specific entry in covariance matrix at row `i`, col `l` from the
% square root information matrix `R` (upper triangular matrix) obtained using 
% Cholesky or QR factorization, and a precomputed inverse diagonal vector `d`,
% where each entry of d(i) = 1 / R(i, i);
function recover(i, l, R, d)
  % Sum over sparse entries of one row
  % eq (6) and eq (7)
  function sum = sum_j(i)
    sum = 0;
    for j = 1:columns(R)
      rij = R(i, j);
      if j != i
        if j > l
          lj = recover(l, j, R, d);
        else
          lj = recover(j, l, R, d);
        endif
        sum += rij * lj;
      endif
    endfor
  endfunction

  if i == l
    % Diagonal entries, eq (6)
    return (d[l] * (d[l] - sum_j(l));
  else
    % Off-diagonal entries, eq(7)
    return (-sum_j(i) * d[i]);
  endif
endfunction

% Recover marginal covariance matrix
function covar = marginal_covar(R, indicies)
  nb_indicies = length(indicies);
  covar = zeros(nb_indicies, nb_indicies);

  % Precalculate inverse diagonal
  function d = recover_precompute(R)
    n = rows(R);
    d = zeros(n);
    for i = 1:n
      d(i) = 1 / R(i, i);
    endfor
  endfunction

  for r = 1:nb_indicies
    for c = 1:nb_indicies
      covar(r, c) = recover(indicies(r), indicies(c));
    endfor
  endfor
endfunction
