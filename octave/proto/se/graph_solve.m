function graph = graph_solve(graph)
  max_iter = 5;

  for i = 1:max_iter
    [H, g, r, param_idx] = graph_eval(graph);
    H = H + 1e-4 * eye(size(H)); % Levenberg-Marquardt Dampening
    dx = linsolve(H, g);
    graph = graph_update(graph, param_idx, dx);
    cost = 0.5 * r' * r
  end
endfunction
