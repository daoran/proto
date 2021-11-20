function C = Exp(phi)
  assert(size(phi) == [3, 1]);
  if (phi < 1e-3)
    C = eye(3) + skew(phi);
  else
    phi_norm = norm(phi);
    phi_skew = skew(phi);
    phi_skew_sq = phi_skew * phi_skew;

    C = eye(3);
    C += (sin(phi_norm) / phi_norm) * phi_skew;
    C += ((1 - cos(phi_norm)) / phi_norm^2) * phi_skew_sq;
  endif
endfunction
