function J = pinhole_equi4_params_jacobian(proj_params, dist_params, p_C)
  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];  % Project 3D point
  x_dist = equi4_distort(dist_params, x);  % Distort point

  J_proj_point = pinhole_point_jacobian(proj_params);
  J_dist_params = equi4_params_jacobian(dist_params, x);

  J = zeros(2, 8);
  J(1:2, 1:4) = pinhole_params_jacobian(proj_params, x_dist);
  J(1:2, 5:8) = J_proj_point * J_dist_params;
endfunction
