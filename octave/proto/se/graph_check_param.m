function graph_check_param(graph, param_id, expected_param_type)
  err_msg = ["param [", num2str(param_id), "] does not exist in graph!"];
  assert(length(graph.params) >= param_id, err_msg);
  assert(isempty(graph.params{param_id}) == 0, err_msg);

  if exist("expected_param_type")
    graph.params{param_id}
    param_type = graph.params{param_id}.type;
    err_msg = ["param.type [", param_type, "] != [", expected_param_type, "] !"];
    assert(graph.params{param_id}.type == expected_param_type, err_msg);
  endif
endfunction
