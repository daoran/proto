function feature = feature_init(feature_id, x)
  assert(all(size(x) == [3, 1]) || all(size(x) == [6, 1]));
  feature.fixed = false;
  feature.type = "feature";
  feature.feature_id = feature_id;
  feature.param = x;
  feature.min_dims = rows(x);
  feature.pose_ids = [];

  if rows(x) == 3
    feature.parameterization = "XYZ";
  else
    feature.parameterization = "INVERSE_DEPTH";
  endif
endfunction
