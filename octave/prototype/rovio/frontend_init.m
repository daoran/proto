function frontend = frontend_init()
  frontend = {};
  frontend.T_C0C1 = eye(4);
  frontend.T_SC0 = eye(4);

  frontend.n_levels = 2;  % Number of image pyramid levels
  frontend.patch_size = 8;  % Edge length of patches in pixels. Must be a multiple of 2
	frontend.n_max = 100;  % Max number of MultilevelPatchFeatures in a MultilevelPatchSet

	frontend.is_valid = zeros(frontend.n_max, 1);
	frontend.features = [];
endfunction
