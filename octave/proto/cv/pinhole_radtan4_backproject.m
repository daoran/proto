function p = pinhole_radtan4_backproject(proj_params, dist_params, z)
  % Extract projection parameters
  fx = proj_params(0);
  fy = proj_params(1);
  cx = proj_params(2);
  cy = proj_params(3);

  % Convert image pixel coordinates to normalized retinal coordintes
  x = [(z(1) - cx) / fx; (z(2) - cy) / fy; 1.0];

  % Undistort
  x = radtan4_undistort(dist_params, x);

  % 3D ray
  p = [x(1); x(2); 1.0];
endfunction
