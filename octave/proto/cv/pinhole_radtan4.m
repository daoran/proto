function camera = pinhole_radtan4(cam_idx, resolution, proj_params, dist_params)
  assert(all(size(resolution) == [2, 1]));
  assert(all(size(proj_params) == [4, 1]));
  assert(all(size(dist_params) == [4, 1]));

  camera.index = cam_idx;
  camera.resolution = resolution;
  camera.proj_params = proj_params;
  camera.dist_params = dist_params;
endfunction
