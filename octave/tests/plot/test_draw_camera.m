addpath(genpath("prototype"));

# Camera pose
C_WC = euler321(deg2rad([-90.0, 0.0, -90.0]));
r_WC = [1.0; 2.0; 3.0];
T_WC = tf(C_WC, r_WC);

debug = false;
if debug
  figure(1);
  hold on;
  draw_camera(T_WC, scale=0.1, style="b-");
  view(3);
  ginput()
end
