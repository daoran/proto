function camera = pinhole_equi4_init(idx, resolution, proj_params, dist_params)
  assert(idx >= 0);
  assert(size(resolution) == [2, 1]);
  assert(length(proj_params) > 0 && columns(proj_params) == 1);
  assert(length(dist_params) > 0 && columns(dist_params) == 1);

  camera.fixed = false;
  camera.type = "camera";
  camera.cam_idx = idx;
  camera.resolution = resolution;
  camera.proj_model = "pinhole";
  camera.dist_model = "equi4";
  camera.param = [proj_params; dist_params];
  camera.min_dims = 8;

  camera.project = @pinhole_equi4_project;
  camera.backproject = @pinhole_equi4_backproject;
  camera.J_proj = @pinhole_equi4_project_jacobian;
  camera.J_param = @pinhole_equi4_params_jacobian;
endfunction
