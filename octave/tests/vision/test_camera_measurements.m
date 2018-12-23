addpath(genpath("prototype"));

# Camera properties
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 90;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2.0;
cy = image_height / 2.0;
K = pinhole_K([fx, fy, cx, cy]);

# Camera pose
C_WC = euler321(deg2rad([-90.0, 0.0, -90.0]));
r_WC = [1.0; 2.0; 3.0];
T_WC = tf(C_WC, r_WC);

# Points in world frame
points_W = 10.0 * rand(3, 100);

# Get camera measurements
[z_out, points_out] = camera_measurements(K, resolution, T_WC, points_W);
assert(rows(z_out) == 2);
assert(columns(z_out) > 0);
assert(rows(points_out) == 3);
assert(columns(points_out) > 0);
assert(columns(z_out) == columns(points_out));

# Plot
debug = false;
if debug
  figure(1);
  hold on;
  draw_points(points_W, color="b");
  draw_camera(T_WC, scale=0.1, style="r-");
  for i = 1:columns(points_out)
    p_W = points_out(1:3, i);
    plot3([r_WC(1), p_W(1)], [r_WC(2), p_W(2)], [r_WC(3), p_W(3)], 'r-');
  end
  view(3);
  ginput();
end
