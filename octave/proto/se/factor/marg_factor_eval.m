function [r, jacs] = marg_factor_eval(marg_factor, params)
  assert(isstruct(marg_factor));

  % Setup
  pose_param_ids = [];
  sb_param_ids = [];
  camera_param_ids = [];
  exts_param_ids = [];
  feature_param_ids = [];

  pose_param_size = 0;
  sb_param_size = 0;
  camera_param_size = 0;
  exts_param_size = 0;
  feature_param_size = 0;

  % Track parameters
  for i = 1:length(marg_factor.factors)
    factor = marg_factor.factors{i};
    % params = graph_get_params(graph, factor.param_ids);
    factor

    for j = 1:length(factor.param_ids)
      param = params{j};
      if strcmp(param.type, "pose")
        pose_param_ids = [pose_param_ids, param.id];
        pose_param_size = param.min_dims;
      elseif strcmp(param.type, "sb")
        sb_param_ids = [sb_param_ids, param.id];
        sb_param_size = param.min_dims;
      elseif strcmp(param.type, "extrinsics")
        exts_param_ids = [exts_param_ids, param.id];
        exts_param_size = param.min_dims;
      elseif strcmp(param.type, "camera")
        camera_param_ids = [camera_param_ids, param.id];
        camera_param_size = param.min_dims;
      elseif strcmp(param.type, "feature")
        feature_param_ids = [feature_param_ids, param.id];
        feature_param_size = param.min_dims;
      endif
    endfor
  endfor
  pose_param_ids = unique(pose_param_ids);
  sb_param_ids = unique(sb_param_ids);
  exts_param_ids = unique(exts_param_ids);
  camera_param_ids = unique(camera_param_ids);
  feature_param_ids = unique(feature_param_ids);

  % Assign global parameter order
  nb_params = 0;
  nb_params += length(pose_param_ids);
  nb_params += length(sb_param_ids);
  nb_params += length(exts_param_ids);
  nb_params += length(camera_param_ids);
  nb_params += length(feature_param_ids);

  param_idx = {};
  col_idx = 1;
  for i = 1:length(pose_param_ids)
    param_idx{pose_param_ids(i)} = col_idx;
    col_idx += pose_param_size;
  endfor
  for i = 1:length(sb_param_ids)
    param_idx{sb_param_ids(i)} = col_idx;
    col_idx += sb_param_size;
  endfor
  for i = 1:length(exts_param_ids)
    param_idx{exts_param_ids(i)} = col_idx;
    col_idx += exts_param_size;
  endfor
  for i = 1:length(camera_param_ids)
    param_idx{camera_param_ids(i)} = col_idx;
    col_idx += camera_param_size;
  endfor
  for i = 1:length(feature_param_ids)
    param_idx{feature_param_ids(i)} = col_idx;
    col_idx += feature_param_size;
  endfor

  % Form Hessian
  param_size = 0;
  param_size += length(pose_param_ids) * pose_param_size;
  param_size += length(sb_param_ids) * sb_param_size;
  param_size += length(exts_param_ids) * exts_param_size;
  param_size += length(camera_param_ids) * camera_param_size;
  param_size += length(feature_param_ids) * feature_param_size;
  H = zeros(param_size, param_size);
	g = zeros(param_size, 1);
	residuals = [];

  for k = 1:length(marg_factor.factors)
    factor = marg_factor.factors{k};
    factor_params = {};
    for x = 1:length(factor.param_ids)
      factor_params{x} = params{factor.param_ids(x)};
    end
    [r, jacobians] = factor.eval(factor, factor_params);
		residuals = [residuals; r];

    for i = 1:length(factor_params)
      idx_i = param_idx{factor_params{i}.id};
      size_i = factor_params{i}.min_dims;
      J_i = jacobians{i};

      for j = i:length(params)
        idx_j = param_idx{factor_params{i}.id};
        size_j = factor_params{i}.min_dims;
        J_j = jacobians{i};

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

  % imagesc(H);
  % ginput();

  r = zeros(1, 1);
  jacs = zeros(1, 1);
endfunction
