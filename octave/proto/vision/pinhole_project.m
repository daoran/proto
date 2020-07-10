function z_hat = pinhole_project(K, T_WC, p_W)
  assert(size(K) == [3, 3]);
  assert(size(T_WC) == [4, 4]);
  assert(size(p_W) == [3, 1]);

  T_CW = tf_inv(T_WC);
  hp_W = homogeneous(p_W);
  hp_C = T_CW * hp_W;
  p_C = dehomogeneous(hp_C);
  x = K * p_C;

  z_hat = zeros(2, 1);
  z_hat(1) = x(1) / x(3);
  z_hat(2) = x(2) / x(3);
endfunction
