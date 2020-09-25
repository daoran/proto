function J = pinhole_radtan4_params_jacobian(proj_params, dist_params, p_C)
  % Projection params
  fx = proj_params(1);
  fy = proj_params(2);
  cx = proj_params(3);
  cy = proj_params(4);

  % Distortion params
  k1 = dist_params(1);
  k2 = dist_params(2);
  p1 = dist_params(3);
  p2 = dist_params(4);

  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];       % Project 3D point
  x_dist = radtan4_distort(dist_params, x);  % Distort point

  J_proj_point = pinhole_point_jacobian(proj_params);
  J_dist_params = radtan4_params_jacobian(dist_params, x);

  J = zeros(2, 8);
  J(1:2, 1:4) = pinhole_params_jacobian(proj_params, p_C);
  J(1:2, 5:8) = J_proj_point * J_dist_params;
endfunction
