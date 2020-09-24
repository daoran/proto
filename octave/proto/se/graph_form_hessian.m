function [H, param_idx] = graph_form_hessian(graph)
  pose_param_ids = [];
  landmark_param_ids = [];
  camera_param_ids = [];

  % Track parameters
  for i = 1:length(graph.factors)
    factor = graph.factors{i};
    params = graph_get_params(graph, factor.param_ids);

    for j = 1:length(params)
      param = params{j};
      if strcmp(param.type, "pose")
        pose_param_ids = [pose_param_ids, param.id];
      elseif strcmp(param.type, "landmark")
        landmark_param_ids = [landmark_param_ids, param.id];
      elseif strcmp(param.type, "camera")
        camera_param_ids = [camera_param_ids, param.id];
      endif
    endfor
  endfor
  pose_param_ids = unique(pose_param_ids);
  landmark_param_ids = unique(landmark_param_ids);
  camera_param_ids = unique(camera_param_ids);

  % Assign global parameter order
  nb_params = 0;
  nb_params += length(pose_param_ids);
  nb_params += length(landmark_param_ids);
  nb_params += length(camera_param_ids);

  param_idx = zeros(nb_params, 1);  % (row: param_id, col: col_idx)
  col_idx = 1;
  for i = 1:length(pose_param_ids)
    param_idx(pose_param_ids(i)) = col_idx;
    col_idx += 6;
  endfor
  for i = 1:length(landmark_param_ids)
    param_idx(landmark_param_ids(i)) = col_idx;
    col_idx += 3;
  endfor
  for i = 1:length(camera_param_ids)
    param_idx(camera_param_ids(i)) = col_idx;
    col_idx += 3;
  endfor

  % Form Hessian
  param_size = 0;
  param_size += length(pose_param_ids) * 6;
  param_size += length(landmark_param_ids) * 3;
  param_size += length(camera_param_ids) * 8;
  H = zeros(param_size, param_size);

  for k = 1:length(graph.factors)
    factor = graph.factors{k};
    params = graph_get_params(graph, factor.param_ids);
    [r, jacobians] = ba_factor_eval(factor, params);

    for i = 1:length(params)
      idx_i = param_idx(params{i}.id);
      size_i = params{i}.min_dims;
      J_i = jacobians{i};

      for j = i:length(params)
        idx_j = param_idx(params{j}.id);
        size_j = params{j}.min_dims;
        J_j = jacobians{j};

        if i == j
          % Diagonal
          rs = idx_i;
          re = idx_i + (size_i - 1);
          cs = idx_j;
          ce = idx_j + (size_j - 1);
          H(rs:re, cs:ce) += J_i' * J_j;
        else
          % Off-Diagonal
          rs = idx_i;
          re = idx_i + (size_i - 1);
          cs = idx_j;
          ce = idx_j + (size_j - 1);
          H(rs:re, cs:ce) += J_i' * J_j;

          rs = idx_j;
          re = idx_j + (size_j - 1);
          cs = idx_i;
          ce = idx_i + (size_i - 1);
          H(rs:re, cs:ce) += (J_i' * J_j)';
        endif
      endfor % Iterate params - j
    endfor  % Iterate params - i
  endfor  % Iterate factors
endfunction
