addpath(genpath("prototype"));

fx = 1.0;
fy = 2.0;
cx = 3.0;
cy = 4.0;
intrinsics = [fx, fy, cx, cy];
K = pinhole_K(intrinsics);

expected = [1.0, 0.0, 3.0;
            0.0, 2.0, 4.0;
            0.0, 0.0, 1.0];
assert(isequal(expected, K));
