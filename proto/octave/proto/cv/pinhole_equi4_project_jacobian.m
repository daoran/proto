function J = pinhole_equi4_project_jacobian(proj_params, dist_params, p_C)
  % Project 3D point
  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];

  % Jacobian
  J_proj = [1 / p_C(3), 0, -p_C(1) / p_C(3)**2;
            0, 1 / p_C(3), -p_C(2) / p_C(3)**2];
  J_dist_point = equi4_point_jacobian(dist_params, x);
  J_proj_point = pinhole_point_jacobian(proj_params);
  J = J_proj_point * J_dist_point * J_proj;
end
