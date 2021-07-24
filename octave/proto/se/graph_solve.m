function graph = graph_solve(graph)
  max_iter = 20;
  lambda = 1e4;

  [H, g, r, param_idx] = graph_eval(graph);
  cost_prev = 0.5 * r' * r;

  for i = 1:max_iter
    H = H + lambda * eye(size(H)); % Levenberg-Marquardt Dampening
    dx = H \ g;

    graph = graph_update(graph, param_idx, dx);
    [H, g, r, param_idx] = graph_eval(graph);
    cost = 0.5 * r' * r;
    dcost = cost_prev - cost;
    printf("cost: %.2e, dcost = %.2e, lambda: %.2e\n", cost, dcost, lambda);

    % Termination criteria
    % -- Cost is low?
    if cost < 1e-3
      printf("Cost < %.2e! terminating!\n", 1e-3);
      break;
    endif
    % -- Convergence speed too low?
    if dcost < 1e-2
      printf("Convergence speed < %.2e terminating!\n", 1e-2);
      break;
    endif

    % Update
    if dcost > 0
      lambda /= 5.0;
    else
      lambda *= 5.0;
    endif
    cost_prev = cost;
  end
endfunction
