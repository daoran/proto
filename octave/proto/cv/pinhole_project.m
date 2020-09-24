function z = pinhole_project(camera, T_WC, p_W)
  assert(size(T_WC) == [4, 4]);
  assert(size(p_W) == [3, 1]);

  T_CW = inv(T_WC);
  p_C = tf_point(T_CW, p_CW);

  fx = camera.param(1);
  fy = camera.param(2);
  cx = camera.param(3);
  cy = camera.param(4);

  x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];  % Project
  z = [fx * x(1) + cx; fy * x(2) + cy];    % Scale and center
endfunction
