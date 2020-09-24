function landmark = landmark_init(lm_id, x)
  assert(all(size(x) == [3, 1]));
  landmark.type = "landmark";
  landmark.lm_id = lm_id;
  landmark.param = x;
  landmark.min_dims = 3;
endfunction
