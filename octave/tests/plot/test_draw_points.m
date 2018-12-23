addpath(genpath("prototype"));

points_W = rand(3, 100);

debug = false;
if debug
  figure(1);
  hold on;
  draw_points(points_W, color="r");
  xlabel("x");
  ylabel("y");
  zlabel("z");
  view(3);
  ginput()
end
