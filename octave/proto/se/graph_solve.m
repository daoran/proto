function graph = graph_solve(graph)
  % Settings
  max_iter = 20;
  lambda = 1e4;

  % Calculate initial cost
  [H, g, r, param_idx] = graph_eval(graph);
  cost_prev = 0.5 * r' * r;

  for i = 1:max_iter
    % Levenberg-Marquardt
    H = H + lambda * eye(size(H));
    % dx = H \ g;
    dx = linsolve(H, g);

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

      % Restore previous estimate if dcost is -ve and terminating
      if dcost < 0
        graph = graph_update(graph, param_idx, -dx);
      endif

      break;
    endif

    % Update
    if dcost > 0
      lambda /= 5.0;
    else
      lambda *= 6.0;
      graph = graph_update(graph, param_idx, -dx);
      [H, g, r, param_idx] = graph_eval(graph);
    endif
    cost_prev = cost;
  end
endfunction
