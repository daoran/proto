function [H, g, residuals, param_idx] = graph_eval(graph)
  pose_param_ids = [];
  exts_param_ids = [];
  sb_param_ids = [];
  feature_param_ids = [];
  camera_param_ids = [];

  pose_param_size = 0;
  exts_param_size = 0;
  sb_param_size = 0;
  feature_param_size = 0;
  camera_param_size = 0;

  % Track parameters
  for i = 1:length(graph.factors)
    factor = graph.factors{i};
    params = graph_get_params(graph, factor.param_ids);

    for j = 1:length(params)
      param = params{j};
      if strcmp(param.type, "pose")
        pose_param_ids = [pose_param_ids, param.id];
        pose_param_size = param.min_dims;
      elseif strcmp(param.type, "extrinsics")
        exts_param_ids = [exts_param_ids, param.id];
        exts_param_size = param.min_dims;
      elseif strcmp(param.type, "sb")
        sb_param_ids = [sb_param_ids, param.id];
        sb_param_size = param.min_dims;
      elseif strcmp(param.type, "feature")
        feature_param_ids = [feature_param_ids, param.id];
        feature_param_size = param.min_dims;
      elseif strcmp(param.type, "camera")
        camera_param_ids = [camera_param_ids, param.id];
        camera_param_size = param.min_dims;
      endif
    endfor
  endfor
  pose_param_ids = unique(pose_param_ids);
  exts_param_ids = unique(exts_param_ids);
  sb_param_ids = unique(sb_param_ids);
  feature_param_ids = unique(feature_param_ids);
  camera_param_ids = unique(camera_param_ids);

  % Assign global parameter order
  nb_params = 0;
  nb_params += length(pose_param_ids);
  nb_params += length(exts_param_ids);
  nb_params += length(sb_param_ids);
  nb_params += length(feature_param_ids);
  nb_params += length(camera_param_ids);

  % printf("pose_param_size: %d\n", pose_param_size);
  % printf("exts_param_size: %d\n", exts_param_size);
  % printf("sb_param_size: %d\n", sb_param_size);
  % printf("feature_param_size: %d\n", feature_param_size);
  % printf("camera_param_size: %d\n", camera_param_size);

  % printf("nb_pose_params: %d\n", length(pose_param_ids));
  % printf("nb_exts_params: %d\n", length(exts_param_ids));
  % printf("nb_sb_params: %d\n", length(sb_param_ids));
  % printf("nb_feature_params: %d\n", length(feature_param_ids));
  % printf("nb_camera_params: %d\n", length(camera_param_ids));

  param_idx = {};
  col_idx = 1;
  for i = 1:length(pose_param_ids)
    param_idx{pose_param_ids(i)} = col_idx;
    col_idx += pose_param_size;
  endfor
  for i = 1:length(exts_param_ids)
    param_idx{exts_param_ids(i)} = col_idx;
    col_idx += exts_param_size;
  endfor
  for i = 1:length(sb_param_ids)
    param_idx{sb_param_ids(i)} = col_idx;
    col_idx += sb_param_size;
  endfor
  for i = 1:length(feature_param_ids)
    param_idx{feature_param_ids(i)} = col_idx;
    col_idx += feature_param_size;
  endfor
  for i = 1:length(camera_param_ids)
    param_idx{camera_param_ids(i)} = col_idx;
    col_idx += camera_param_size;
  endfor

  % Form Hessian
  param_size = 0;
  param_size += length(pose_param_ids) * pose_param_size;
  param_size += length(exts_param_ids) * exts_param_size;
  param_size += length(sb_param_ids) * sb_param_size;
  param_size += length(feature_param_ids) * feature_param_size;
  param_size += length(camera_param_ids) * camera_param_size;
  H = zeros(param_size, param_size);
	g = zeros(param_size, 1);
	residuals = [];

  for k = 1:length(graph.factors)
    factor = graph.factors{k};
    params = graph_get_params(graph, factor.param_ids);
    [r, jacobians] = factor.eval(factor, params);
		residuals = [residuals; r];

    for i = 1:length(params)
      idx_i = param_idx{params{i}.id};
      size_i = params{i}.min_dims;
      J_i = jacobians{i};

      for j = i:length(params)
        idx_j = param_idx{params{j}.id};
        size_j = params{j}.min_dims;
        J_j = jacobians{j};

				rs = idx_i;
				re = idx_i + (size_i - 1);
				cs = idx_j;
				ce = idx_j + (size_j - 1);

        if i == j
          % Diagonal
          H(rs:re, cs:ce) += J_i' * J_j;
        else
          % Off-Diagonal
          H(rs:re, cs:ce) += J_i' * J_j;
          H(cs:ce, rs:re) += (J_i' * J_j)';
        endif
      endfor % Iterate params - j

			rs = idx_i;
			re = idx_i + (size_i - 1);
			g(rs:re) += -J_i' * r;
    endfor  % Iterate params - i
  endfor  % Iterate factors
endfunction
