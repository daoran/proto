function camera = camera_init(resolution, fov, D=zeros(4, 1))
  fx = focal_length(resolution(1), fov);
  fy = focal_length(resolution(2), fov);
  cx = resolution(1) / 2.0;
  cy = resolution(2) / 2.0;
  intrinsics = [fx, fy, cx, cy];

  camera.resolution = resolution;
  camera.K = pinhole_K(intrinsics);
  camera.D = D;
endfunction
