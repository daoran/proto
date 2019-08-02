addpath(genpath("proto"));

image_width = 480.0;
fov = 60.0;
fx = focal_length(image_width, fov);
assert(abs(fx - 415.69219) < 1e-5);
