function check_factor_jacobians(factor_eval, factor, params, param_idx, jac_name, step_size, threshold)
  % Calculate baseline
  [r, jacs] = factor_eval(factor, params);

  % Numerical diff
  fdiff = zeros(rows(r), params{param_idx}.min_dims);
  for i = 1:params{param_idx}.min_dims
    % Perturb and evaluate
    params_fwd = params;
    params_fwd{param_idx}.param = tf_param(perturb_pose(tf(params{param_idx}.param), step_size, i));
    [r_fwd, _] = factor_eval(factor, params_fwd);

    % Forward finite difference
    fdiff(:, i) = (r_fwd - r) / step_size;
  endfor

  jacs{param_idx}
  fdiff
  check_jacobian(jac_name, fdiff, jacs{param_idx}, threshold, true);
endfunction
