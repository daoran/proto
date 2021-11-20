function [graph, factor_id] = graph_add_factor(graph, factor)
  assert(isfield(factor, "param_ids"));
  assert(length(factor.param_ids) > 0);

  % Check parameter id exists
  for i = 1:length(factor.param_ids)
    graph_check_param(graph, factor.param_ids(i));
  endfor

  % Add parameter to graph
  factor_id = length(graph.factors) + 1;
  factor.id = factor_id;
  graph.factors{length(graph.factors) + 1} = factor;

  % BA factor
  if strcmp(factor.type, "ba_factor")
    pose_id = factor.param_ids(1);
    feature_id = factor.param_ids(2);
    feature_pose_ids = graph.params{feature_id}.pose_ids;
    graph.params{feature_id}.pose_ids = [feature_pose_ids, pose_id];
  elseif strcmp(factor.type, "cam_factor")
    pose_id = factor.param_ids(1);
    feature_id = factor.param_ids(3);
    feature_pose_ids = graph.params{feature_id}.pose_ids;
    graph.params{feature_id}.pose_ids = [feature_pose_ids, pose_id];
  endif
endfunction
