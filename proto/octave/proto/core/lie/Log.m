function rvec = Log(C)
  assert(size(C) == [3, 3]);
  % phi = acos((trace(C) - 1) / 2);
  % u = skew_inv(C - C') / (2 * sin(phi));
  % rvec = phi * u;

  C11 = C(1, 1); C12 = C(1, 2); C13 = C(1, 3);
  C21 = C(2, 1); C22 = C(2, 2); C23 = C(2, 3);
  C31 = C(3, 1); C32 = C(3, 2); C33 = C(3, 3);

  tr = trace(C);
  if (tr + 1.0 < 1e-10)
    if (abs(C33 + 1.0) > 1e-5)
      rvec = (pi / sqrt(2.0 + 2.0 * C33)) * [C13; C23; 1.0 + C33];
    elseif (abs(C22 + 1.0) > 1e-5)
      rvec = (pi / sqrt(2.0 + 2.0 * C22)) * [C12; 1.0 + C22; C32];
    else
      rvec = (pi / sqrt(2.0 + 2.0 * C11)) * [1.0 + C11; C21; C31];
    endif

  else
    tr_3 = tr - 3.0;  % always negative
    if tr_3 < -1e-7
      theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    else
      % when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      % use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      % see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0;
    endif
    rvec = magnitude * [C32 - C23; C13 - C31; C21 - C12];
  endif

endfunction
