addpath(genpath("proto"));
graphics_toolkit("fltk");

grid = aprilgrid_init();

p_F = [];
for id=0:(grid.rows*grid.cols)-1
  points = grid.object_points{id + 1};
  p_F = [p_F, points];
endfor
hp_F = homogeneous(p_F);

rpy = deg2rad([90.0, 0.0, -90.0]);
C_WF = euler321(rpy);
T_WF = tf(C_WF, zeros(3, 1));
p_WF = dehomogeneous(T_WF * hp_F);

figure();
scatter3(p_WF(1, :), p_WF(2, :), p_WF(3, :), "r", "filled");
view(3);
axis("equal");
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
ginput();
