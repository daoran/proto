function camera = camera_init(cam_idx, resolution,
                              proj_model, dist_model,
                              proj_params, dist_params,
                              proj_fn, J_proj_fn, J_param_fn)
  assert(cam_idx >= 0);
  assert(size(resolution) == [2, 1]);
  assert(length(proj_params) > 0 && columns(proj_params) == 1);
  assert(length(dist_params) > 0 && columns(dist_params) == 1);

  camera.type = "camera";
  camera.cam_idx = cam_idx;
  camera.resolution = resolution;
  camera.proj_model = proj_model;
  camera.dist_model = dist_model;
  camera.param = [proj_params; dist_params];
  camera.min_dims = 8;
endfunction
