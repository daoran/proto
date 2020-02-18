function features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_features)
  features = zeros(nb_features, 3);
  for i = 1:nb_features
    features(i, 1) = randf(x_bounds);
    features(i, 2) = randf(y_bounds);
    features(i, 3) = randf(z_bounds);
	endfor
endfunction
