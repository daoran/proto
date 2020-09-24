function graph = graph_update(graph, param_idx, dx)
  for i = 1:rows(param_idx)
    param_id = i;

    param = graph.param{param_id};
    idx = param_idx(i);

    % Update pose
    if strcmp(param.type, "pose")
      dalpha = dx(idx:idx+2);
      dq = quat_delta(dalpha);
      quat = param.param(1:4);
      param.param(1:4) = quat_mul(dq, quat);

      dr = dx(idx+3:idx+5);
      trans = param.param(1:4);
      param.param(5:7) = trans + dr;
    else
      param.param = param.param + dx(idx:(idx+param.min_dims-1));
    end
  endfor
endfunction
