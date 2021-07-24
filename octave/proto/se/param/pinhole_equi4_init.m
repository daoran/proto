function cam = pinhole_equi4_init(idx, resolution, proj_params, dist_params)
  assert(idx >= 0);
  assert(size(resolution) == [2, 1]);
  assert(length(proj_params) > 0 && columns(proj_params) == 1);
  assert(length(dist_params) > 0 && columns(dist_params) == 1);

  cam.type = "camera";
  cam.cam_idx = idx;
  cam.resolution = resolution;
  cam.proj_model = "pinhole";
  cam.dist_model = "equi4";
  cam.param = [proj_params; dist_params];
  cam.min_dims = 8;

  cam.project = @pinhole_equi4_project;
  cam.J_proj = @pinhole_equi4_project_jacobian;
  cam.J_param = @pinhole_equi4_params_jacobian;
endfunction
