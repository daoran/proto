function point = radtan4_undistort(dist_params, p0)
  assert(length(dist_params) == 4);

  % Distortion parameters
  k1 = dist_params(1);
  k2 = dist_params(2);
  p1 = dist_params(3);
  p2 = dist_params(4);

  % Undistort
  p = p0;
  max_iter = 5;

  for i = 1:max_iter
    % Error
    p_distorted = radtan4_distort(dist_params, p);
    J = radtan4_point_jacobian(dist_params, p);
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
