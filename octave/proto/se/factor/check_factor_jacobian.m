function check_factor_jacobian(factor, params, param_idx, jac_name, step_size, threshold)
  % Calculate baseline
  [r, jacs] = factor.eval(factor, params);

  % Numerical diff
  J_numdiff = zeros(rows(r), params{param_idx}.min_dims);
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
    [r_fwd, _] = factor.eval(factor, params_fwd);

    % Forward finite difference
    J_numdiff(:, i) = (r_fwd - r) / step_size;
  endfor

  J_analytical = jacs{param_idx};
  check_jacobian(jac_name, J_numdiff, J_analytical, threshold, true);
endfunction
