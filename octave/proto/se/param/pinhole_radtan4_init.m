function cam = pinhole_radtan4_init(idx, resolution, proj_params, dist_params)
  assert(idx >= 0);
  assert(size(resolution) == [2, 1]);
  assert(length(proj_params) > 0 && columns(proj_params) == 1);
  assert(length(dist_params) > 0 && columns(dist_params) == 1);

  cam.type = "camera";
  cam.cam_idx = idx;
  cam.resolution = resolution;
  cam.proj_model = "pinhole";
  cam.dist_model = "radtan4";
  cam.param = [proj_params; dist_params];
  cam.min_dims = 8;

  cam.project = @pinhole_radtan4_project;
  cam.backproject = @pinhole_radtan4_backproject;
  cam.J_proj = @pinhole_radtan4_project_jacobian;
  cam.J_param = @pinhole_radtan4_params_jacobian;
endfunction
