addpath(genpath("proto"));

image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.001; 1e-4; 1e-4];

index = 0;
camera = pinhole_radtan4(index, resolution, proj_params, dist_params);
