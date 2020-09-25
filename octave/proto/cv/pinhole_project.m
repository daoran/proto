function z = pinhole_project(proj_params, T_WC, p_W)
  assert(length(proj_params) == 4);
  assert(size(T_WC) == [4, 4]);
  assert(size(p_W) == [3, 1]);

  T_CW = inv(T_WC);
  p_C = tf_point(T_CW, p_W);

  fx = proj_params(1);
  fy = proj_params(2);
  cx = proj_params(3);
  cy = proj_params(4);

  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];  % Project
  z = [fx * x(1) + cx; fy * x(2) + cy];    % Scale and center
endfunction
