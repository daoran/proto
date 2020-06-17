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
