function graph = graph_update(graph, param_idx, dx)
  for i = 1:length(graph.params)
    param = graph.params{i};
    if param.id > length(param_idx) || isempty(param_idx{param.id})
      continue;
    endif

		% Update parameter
    idx = param_idx{param.id};
    if (strcmp(param.type, "pose") == 1 || strcmp(param.type, "extrinsics"))
      param = pose_update(param, dx(idx:idx+5));
    else
      param.param += dx(idx:(idx+param.min_dims-1));
    end

		% Replace the parameter with new updated
		graph.params{i} = param;
  endfor
endfunction
