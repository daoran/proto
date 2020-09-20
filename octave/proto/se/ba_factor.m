function factor = ba_factor(camera_params, param_ids, z, covar=eye(2))
  factor.camera_params = camera_params;
  factor.param_ids = param_ids;
  factor.z = z;
  factor.covar = covar;
endfunction
