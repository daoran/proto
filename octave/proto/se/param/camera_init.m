function camera = camera_init(cam_idx, resolution, proj_model, dist_model, proj_params, dist_params)
  camera.type = "camera";
  camera.cam_idx = cam_idx;
  camera.resolution = resolution;
  camera.proj_model = proj_model;
  camera.dist_model = dist_model;
  camera.param = [proj_params; dist_params];
  camera.min_dims = 8;
endfunction
