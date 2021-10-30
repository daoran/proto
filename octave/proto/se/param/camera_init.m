function camera = camera_init(cam_idx, resolution,
                              proj_model, dist_model,
                              proj_params, dist_params)
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

  if strcmp(proj_model, "pinhole") && strcmp(dist_model, "radtan4")
    camera.project = @pinhole_radtan4_project;
    camera.backproject = @pinhole_radtan4_backproject;
    camera.J_proj = @pinhole_radtan4_project_jacobian;
    camera.J_param = @pinhole_radtan4_params_jacobian;
  elseif strcmp(proj_model, "pinhole") && strcmp(dist_model, "equi4")
    camera.project = @pinhole_equi4_project;
    camera.backproject = @pinhole_equi4_backproject;
    camera.J_proj = @pinhole_equi4_project_jacobian;
    camera.J_param = @pinhole_equi4_params_jacobian;
  endif
endfunction
