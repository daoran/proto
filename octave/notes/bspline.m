function s = deboor(k, x, t, c, p)
  % Evaluates S(x).
  %
  % Args
  % ----
  % k: index of knot interval that contains x
  % x: position
  % t: array of knot positions, needs to be padded as described above
  % c: array of control points
  % p: degree of B-spline
  % """
  d = zeros(p + 1);
  for j = 1:p+1
    d(j) = c(j + k - p)
  endfor

  for r = 1:p+1
    for j = p:r-1:-1
      alpha = (x - t(j+k-p)) / (t(j+1+k-r) - t(j+k-p));
      d(j) = (1.0 - alpha) * d(j-1) + alpha * d(j);
    endfor
  endfor

  d(p)
endfunction

function N = coeffs(n, p, m, i, knots)
  N = 0;
  u = 2;

  % Special case
  u_0 = knots(1);
  u_m = knots(end);
  if u == u_0
    N = 1.0;
    return;
  elseif u == u_m
    N = 1.0;
    return;
  endif

  % When u is between u_0 and u_m
  % N(k) = 1.0;
  for d = 1:p
    N
  endfor
endfunction

# Signal
f = 1.0;
t = linspace(0.0, 1.0, 15);
y = sin(2 * pi * f * t);

n = 15;
p = 3;
m = n + p + 1;
u = 0.0;
knots = t;
coeffs(n, p, m, u, knots)

% # Plot
% figure();
% plot(t, y, "r-", "linewidth", 2.0);
% ginput();
