function feature = feature_init(f_id, x)
  assert(all(size(x) == [3, 1]));
  feature.type = "feature";
  feature.f_id = f_id;
  feature.param = x;
  feature.min_dims = 6;
endfunction
