function graph = graph_init()
  % Parameters and factors
  graph.params = {};
  graph.factors = {};

  % Parameter ids
  graph.camera_param_ids = [];
  graph.pose_param_ids = [];
  graph.sb_param_ids = [];
  graph.exts_param_ids = [];
  graph.feature_param_ids = [];
endfunction
