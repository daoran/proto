function z = pinhole_radtan4_project(proj_params, dist_params, p)
  assert(all(size(proj_params) == [4, 1]));
  assert(all(size(dist_params) == [4, 1]));
  assert(all(size(p) == [3, 1]));

	% Project
  x = [p(1) / p(3); p(2) / p(3)];

	% Distort
  k1 = dist_params(1);
  k2 = dist_params(2);
  p1 = dist_params(3);
  p2 = dist_params(4);
  x_dist = radtan4_distort(k1, k2, p1, p2, x);

	% Scale and center to image plane
  fx = proj_params(1);
  fy = proj_params(2);
  cx = proj_params(3);
  cy = proj_params(4);
  z = [fx * x_dist(1) + cx; fy * x_dist(2) + cy];
endfunction
