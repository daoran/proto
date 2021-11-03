function [r, jacs] = marg_factor_eval(marg_factor, params)
  assert(isstruct(marg_factor));

  % Setup
  marg_param_ids = marg_factor.marg_param_ids;
  pose_param_ids = [];
  sb_param_ids = [];
  camera_param_ids = [];
  exts_param_ids = [];
  feature_param_ids = [];

  marg_param_sizes = [];
  pose_param_size = 0;
  sb_param_size = 0;
  camera_param_size = 0;
  exts_param_size = 0;
  feature_param_size = 0;

  % Obtain marg param sizes
  for i = 1:length(marg_param_ids)
    param_id = marg_param_ids(i);
    marg_param_sizes = [marg_param_sizes, params{param_id}.min_dims];
  endfor

  % Track parameters
  for i = 1:length(marg_factor.factors)
    factor = marg_factor.factors(i);

    for j = 1:length(factor.param_ids)
      param = params{factor.param_ids(j)};

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
  pose_param_ids = setdiff(pose_param_ids, marg_param_ids);
  sb_param_ids = setdiff(sb_param_ids, marg_param_ids);
  exts_param_ids = setdiff(exts_param_ids, marg_param_ids);
  camera_param_ids = setdiff(camera_param_ids, marg_param_ids);
  feature_param_ids = setdiff(feature_param_ids, marg_param_ids);

  % Assign global parameter order
  nb_params = 0;
  nb_params += length(marg_param_ids);
  nb_params += length(pose_param_ids);
  nb_params += length(sb_param_ids);
  nb_params += length(exts_param_ids);
  nb_params += length(camera_param_ids);
  nb_params += length(feature_param_ids);

  param_indices = {};
  col_idx = 1;
  for i = 1:length(marg_param_ids)
    param_indices{marg_param_ids(i)} = col_idx;
    col_idx += marg_param_sizes(i);
  endfor
  for i = 1:length(pose_param_ids)
    param_indices{pose_param_ids(i)} = col_idx;
    col_idx += pose_param_size;
  endfor
  for i = 1:length(sb_param_ids)
    param_indices{sb_param_ids(i)} = col_idx;
    col_idx += sb_param_size;
  endfor
  for i = 1:length(exts_param_ids)
    param_indices{exts_param_ids(i)} = col_idx;
    col_idx += exts_param_size;
  endfor
  for i = 1:length(camera_param_ids)
    param_indices{camera_param_ids(i)} = col_idx;
    col_idx += camera_param_size;
  endfor
  for i = 1:length(feature_param_ids)
    param_indices{feature_param_ids(i)} = col_idx;
    col_idx += feature_param_size;
  endfor

  % Form Hessian
  param_size = 0;
  param_size += sum(marg_param_sizes);
  param_size += length(pose_param_ids) * pose_param_size;
  param_size += length(sb_param_ids) * sb_param_size;
  param_size += length(exts_param_ids) * exts_param_size;
  param_size += length(camera_param_ids) * camera_param_size;
  param_size += length(feature_param_ids) * feature_param_size;
  H = zeros(param_size, param_size);
	g = zeros(param_size, 1);
	r = [];

  for k = 1:length(marg_factor.factors)
    factor = marg_factor.factors(k);
    for x = 1:length(factor.param_ids)
      factor_params{x} = params{factor.param_ids(x)};
    end
    [residuals, jacobians] = factor.eval(factor, factor_params);
		r = [r; residuals];

    for i = 1:length(factor_params)
      if factor_params{i}.fixed
        continue;
      elseif any(marg_param_ids == factor_params{i}.id)
        continue;
      endif
      idx_i = param_indices{factor_params{i}.id};
      size_i = factor_params{i}.min_dims;
      J_i = jacobians{i};

      for j = i:length(factor_params)
        if factor_params{j}.fixed
          continue;
        elseif any(marg_param_ids == factor_params{j}.id)
          continue;
        endif
        idx_j = param_indices{factor_params{j}.id};
        size_j = factor_params{j}.min_dims;
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
			g(rs:re) += -J_i' * residuals;
    endfor  % Iterate params - i
  endfor  % Iterate factors

  % Perform marginalization
  marg_size = sum(marg_param_sizes);
  remain_size = rows(H) - marg_size;
  [H_marg, g_marg] = schurs_complement(H, g, marg_size, remain_size);

  % figure(1);
  % imagesc(normalize(H));
  %
  % figure(2);
  % imagesc(normalize(H_marg));
  % ginput();

  % % Decompose Hessian back to J
  % eig(H_marg)

  r = zeros(1, 1);
  jacs = zeros(1, 1);
endfunction
