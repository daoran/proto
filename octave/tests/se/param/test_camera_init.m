addpath(genpath("proto"));

cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_model = "pinhole";
dist_model = "radtan4";
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera  = camera_init(cam_idx, resolution,
                      proj_model, dist_model,
                      proj_params, dist_params);

assert(camera.type == "camera");
assert(camera.cam_idx == cam_idx);
assert(camera.resolution == resolution);
assert(camera.proj_model == proj_model);
assert(camera.dist_model == dist_model);
assert(camera.param == [proj_params; dist_params]);
assert(camera.min_dims == 8);
