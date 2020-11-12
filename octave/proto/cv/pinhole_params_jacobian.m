function J = pinhole_params_jacobian(proj_params, x)
  J = [x(1), 0, 1, 0; 0, x(2), 0, 1];
endfunction
