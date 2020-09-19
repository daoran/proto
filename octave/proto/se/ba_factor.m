function factor = ba_factor(camera, p, z, covar=eye(2))
  factor = {};
  factor.camera = camera;
  factor.z = z;
  factor.p = p;
  factor.covar = covar;
  factor.eval = @ba_factor_eval;
endfunction
