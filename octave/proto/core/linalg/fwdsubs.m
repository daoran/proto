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
