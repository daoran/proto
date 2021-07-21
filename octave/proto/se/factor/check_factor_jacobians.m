function check_factor_jacobians(factor_eval, factor, params, param_idx, jac_name, step_size, threshold)
  % Calculate baseline
  [r, jacs] = factor_eval(factor, params);

  % Numerical diff
  fdiff = zeros(rows(r), params{param_idx}.min_dims);
  for i = 1:params{param_idx}.min_dims
    % Perturb and evaluate
    params_fwd = params;
    if strcmp(params_fwd{param_idx}.type, "pose") == 1
      T = tf(params_fwd{param_idx}.param);
      T_fwd = perturb_pose(T, step_size, i);
      params_fwd{param_idx}.param = tf_param(T_fwd);
    else
      params_fwd{param_idx}.param(i) += step_size;
    endif

    % Evaluate
    [r_fwd, _] = factor_eval(factor, params_fwd);

    % Forward finite difference
    fdiff(:, i) = (r_fwd - r) / step_size;
  endfor

  check_jacobian(jac_name, fdiff, jacs{param_idx}, threshold, true);
endfunction
