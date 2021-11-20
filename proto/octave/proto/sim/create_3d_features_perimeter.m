function features = create_3d_features_perimeter(origin, dim, nb_features)
  assert(size(origin) == [3, 1]);
  assert(size(dim) == [3, 1]);
  assert(nb_features > 0);

  % Dimension of the outskirt
  w = dim(1);
  l = dim(2);
  h = dim(3);

  % Features per side
  nb_fps = nb_features / 4.0;

  % Features in the east side
  x_bounds = [origin(1) - w, origin(1) + w];
  y_bounds = [origin(2) + l, origin(2) + l];
  z_bounds = [origin(3) - h, origin(3) + h];
  east_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);

  % Features in the north side
  x_bounds = [origin(1) + w, origin(1) + w];
  y_bounds = [origin(2) - l, origin(2) + l];
  z_bounds = [origin(3) - h, origin(3) + h];
  north_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);

  % Features in the west side
  x_bounds = [origin(1) - w, origin(1) + w];
  y_bounds = [origin(2) - l, origin(2) - l];
  z_bounds = [origin(3) - h, origin(3) + h];
  west_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);

  % Features in the south side
  x_bounds = [origin(1) - w, origin(1) - w];
  y_bounds = [origin(2) - l, origin(2) + l];
  z_bounds = [origin(3) - h, origin(3) + h];
  south_features = create_3d_features(x_bounds, y_bounds, z_bounds, nb_fps);

  % Stack features and return
  features = [east_features; north_features; west_features; south_features];
endfunction
