function J = pinhole_params_jacobian(proj_params, p_C)
  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];       % Project 3D point
  J = [x(1), 0, 1, 0; 0, x(2), 0, 1];
endfunction
