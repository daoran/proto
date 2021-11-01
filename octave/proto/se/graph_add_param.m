function [graph, param_id] = graph_add_param(graph, param)
  param_id = length(graph.params) + 1;
  param.id = param_id;
  graph.params{param_id} = param;

  if strcmp(param.type, "camera")
    graph.camera_param_ids = [graph.camera_param_ids, param_id];
  elseif strcmp(param.type, "pose")
    graph.pose_param_ids = [graph.pose_param_ids, param_id];
  elseif strcmp(param.type, "sb")
    graph.sb_param_ids = [graph.sb_param_ids, param_id];
  elseif strcmp(param.type, "extrinsics")
    graph.exts_param_ids = [graph.exts_param_ids, param_id];
  elseif strcmp(param.type, "feature")
    graph.feature_param_ids = [graph.feature_param_ids, param_id];
  endif
endfunction
