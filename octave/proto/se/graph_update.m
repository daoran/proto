function graph = graph_update(graph, param_idx, dx)
  for i = 1:length(graph.params)
    param = graph.params{i};
    idx = param_idx{param.id};

		% Update parameter
    if strcmp(param.type, "pose")
      param = pose_update(param, dx(idx:idx+5));
    else
      param.param = param.param + dx(idx:(idx+param.min_dims-1));
    end

		% Replace the parameter with new updated
		graph.params{i} = param;
  endfor
endfunction
