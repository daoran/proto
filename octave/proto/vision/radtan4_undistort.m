function point = radtan4_undistort(k1, k2, p1, p2, p0)
  p = p0;
  max_iter = 5;

  for i = 1:max_iter
    % Error
    p_distorted = radtan4_distort(k1, k2, p1, p2, p);
    J = radtan4_point_jacobian(k1, k2, p1, p2, p);
    err = (p0 - p_distorted);

    % Update
    % dp = inv(J' * J) * J' * err;
    dp = pinv(J) * err;
    p = p + dp;

    % Check threshold
    if (err' * err) < 1e-15
      break;
    endif
  endfor

  point = p;
endfunction
