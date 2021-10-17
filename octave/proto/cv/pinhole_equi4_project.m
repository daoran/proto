function z = pinhole_equi4_project(proj_params, dist_params, p)
  assert(all(size(proj_params) == [4, 1]));
  assert(all(size(dist_params) == [4, 1]));
  assert(all(size(p) == [3, 1]));

  % Project
  x = [p(1) / p(3); p(2) / p(3)];

  % Distort
  x_dist = equi4_distort(dist_params, x);

  % Scale and center to image plane
  fx = proj_params(1);
  fy = proj_params(2);
  cx = proj_params(3);
  cy = proj_params(4);
  z = [fx * x_dist(1) + cx; fy * x_dist(2) + cy];
endfunction
