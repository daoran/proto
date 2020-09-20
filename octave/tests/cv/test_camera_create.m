addpath(genpath("proto"));

resolution = [680; 480];
fov = 90.0;
camera = camera_create(resolution, fov);

K = [340.0, 0.0, 340.0;
     0.0, 240.0, 240.0;
     0.0, 0.0, 1.0;];

assert(isequal(camera.resolution, resolution));
assert(isapprox(camera.K, K, 1e-5));
assert(isequal(camera.D, zeros(4, 1)));
