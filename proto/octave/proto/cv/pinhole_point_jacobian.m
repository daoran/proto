function J = pinhole_point_jacobian(proj_params)
  fx = proj_params(1);
  fy = proj_params(2);
  J = [fx, 0; 0, fy];
endfunction
