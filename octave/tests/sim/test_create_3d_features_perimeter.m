addpath(genpath("proto"));
graphics_toolkit("fltk");

origin = [0; 0; 0];
dim = [10; 10; 5];
nb_features = 1000;
features = create_3d_features_perimeter(origin, dim, nb_features);

debug_mode = 0;
if debug_mode
  figure();
  scatter3(features(:, 1), features(:, 2), features(:, 3), 'filled');
  xlabel("x");
  ylabel("y");
  zlabel("z");
  axis('equal');
  ginput();
endif
