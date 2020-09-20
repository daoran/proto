function R = euler321(rpy)
  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);

  R11 = cos(psi) * cos(theta);
  R21 = sin(psi) * cos(theta);
  R31 = -sin(theta);

  R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  R32 = cos(theta) * sin(phi);

  R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  R33 = cos(theta) * cos(phi);

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function x = bwdsubs(U, b)
  % Solving an upper triangular system by back-substitution
  % Input matrix U is an n by n upper triangular matrix
  % Input vector b is n by 1
  % Output vector x is the solution to the linear system
  % U x = b
  assert(columns(U) == rows(b));
  n = rows(b);

  x = zeros(n, 1);
  for j=n:-1:1
    if (U(j, j) == 0)
      error('Matrix is singular!');
    end;
    x(j) = b(j) / U(j, j);
    b(1:j-1) = b(1:j-1) - U(1:j-1, j) * x(j);
  end
endfunction

function x = fwdsubs(L, b)
  % Solving a lower triangular system by forward-substitution
  % Input matrix L is an n by n lower triangular matrix
  % Input vector b is n by 1
  % Output vector x is the solution to the linear system
  % L x = b
  assert(columns(L) == rows(b));
  n = rows(b);

  x = zeros(n, 1);
  for j=1:n
    if (L(j, j) == 0)
      error('Matrix is singular!');
    end;
    x(j) = b(j) / L(j, j);
    b(j+1:n) = b(j+1:n) - L(j+1:n, j) * x(j);
  endfor
endfunction

% rpy = rand(3, 1);
% C = euler321(rpy);
% [L, U, P] = lu(C);
%
% L_inv = inv(L)
%
% b = eye(3);
% L1_inv = fwdsubs(L, b(:, 1), 3);
% L2_inv = fwdsubs(L, b(:, 2), 3);
% L3_inv = fwdsubs(L, b(:, 3), 3);
% Linv = [L1_inv, L2_inv, L3_inv]


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
