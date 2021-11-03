function graph = graph_solve(graph)
  % Settings
  max_iter = 100;
  % lambda = 1e8;
  lambda = 1e-4;

  % Calculate initial cost
  [H, g, r, param_indices] = graph_eval(graph);
  cost_prev = 0.5 * r' * r;
  printf("iter[0] cost: %.2e, dcost = 0.00, lambda: %.2e\n", cost_prev, lambda);

  nb_bad_iters = 0;
  for i = 1:max_iter
    % Levenberg-Marquardt
    H = H + lambda * eye(size(H));
    dx = H \ g;
    % dx = pinv(H) * g;
    % dx = linsolve(H, g);

    % warning('off');
    % H_sparse = sparse(H);
    % dx = pcg(H_sparse, g, 1e-3, 100);
    % warning('on');

    graph = graph_update(graph, param_indices, dx);
    [H, g, r, param_indices] = graph_eval(graph);
    cost = 0.5 * r' * r;
    dcost = cost_prev - cost;
    printf("iter[%d] cost: %.2e, dcost = %.2e, lambda: %.2e\n", i, cost, dcost, lambda);

    % Termination criteria
    % -- Cost is low?
    if cost < 1e-3
      printf("Cost < %.2e! terminating!\n", 1e-3);
      break;
    endif
    % -- Convergence speed too low?
    if dcost < 1e-2
      nb_bad_iters += 1;

      if nb_bad_iters > 3
        printf("Convergence speed < %.2e terminating!\n", 1e-2);
        break;
      endif
    endif

    % Update
    if dcost > 0
      lambda /= 10.0;
      nb_bad_iters = 0;
      cost_prev = cost;
    else
      lambda *= 5.0;
      graph = graph_update(graph, param_indices, -dx);
      [H, g, r, param_indices] = graph_eval(graph);
      cost_prev = 0.5 * r' * r;
    endif
  end
endfunction
