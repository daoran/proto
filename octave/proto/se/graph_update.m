function graph = graph_update(graph, param_idx, dx)
  for i = 1:rows(param_idx)
    param = graph.params{i};
    idx = param_idx(i);

		% Update parameter
    if strcmp(param.type, "pose")
			% Update pose
      dalpha = dx(idx:idx+2);
      dq = quat_delta(dalpha);
      param.param(1:4) = quat_mul(dq, param.param(1:4));

      dr = dx(idx+3:idx+5);
      param.param(5:7) = param.param(5:7)+ dr;
    else
      param.param = param.param + dx(idx:(idx+param.min_dims-1));
    end

		% Replace the parameter with new updated
		graph.params{i} = param;
  endfor
endfunction
