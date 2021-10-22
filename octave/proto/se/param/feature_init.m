function feature = feature_init(id, x)
  assert(all(size(x) == [3, 1]) || all(size(x) == [6, 1]));
  feature.type = "feature";
  feature.id = id;
  feature.param = x;
  feature.min_dims = rows(x);
  if rows(x) == 3
    feature.parameterization = "XYZ";
  else
    feature.parameterization = "INVERSE_DEPTH";
  endif
endfunction
