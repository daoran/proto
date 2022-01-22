function retval = check_factor_jacobian(factor, params, param_idx, jac_name, step_size, threshold)
  % Calculate baseline
  [r, jacs] = factor.eval(factor, params);

  % Numerical diff
  J_numdiff = zeros(rows(r), params{param_idx}.min_dims);
  for i = 1:params{param_idx}.min_dims
    % Forward perturb and evaluate
    params_fwd = params;
    if strcmp(params_fwd{param_idx}.type, "pose") == 1
      T = tf(params_fwd{param_idx}.param);
      T_fwd = perturb_pose(T, step_size / 2.0, i);
      params_fwd{param_idx}.param = tf_param(T_fwd);
    else
      params_fwd{param_idx}.param(i) += step_size / 2.0;
    endif
    % Evaluate
    [r_fwd, _] = factor.eval(factor, params_fwd);

    % Backward perturb and evaluate
    params_bwd = params;
    if strcmp(params_bwd{param_idx}.type, "pose") == 1
      T = tf(params_bwd{param_idx}.param);
      T_bwd = perturb_pose(T, -step_size / 2.0, i);
      params_bwd{param_idx}.param = tf_param(T_bwd);
    else
      params_bwd{param_idx}.param(i) -= step_size / 2.0;
    endif
    % Evaluate
    [r_bwd, _] = factor.eval(factor, params_bwd);

    % Forward finite difference
    J_numdiff(:, i) = (r_fwd - r_bwd) / step_size;
  endfor

  J_analytical = jacs{param_idx};
  retval = check_jacobian(jac_name, J_numdiff, J_analytical, threshold, true);
endfunction
