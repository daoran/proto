addpath(genpath("proto"));

% Setup camera pose T_WC
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC = tf(rot, trans);
data = [rot; trans];
cam_pose = pose_t(0, data);

% Setup cam0
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
cam_params = camera_t(cam_idx, resolution,
                      proj_model, dist_model,
                      proj_params, dist_params);

% Setup landmark
lm = landmark_t(0, [10; rand(2, 1)]);
p_C = tf_point(inv(T_WC), lm.data);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);

% Setup graph
graph.params = {};
graph.factors = {};

% -- Add parameters to graph
graph.params{1} = cam_pose;
graph.params{2} = lm;
graph.params{3} = cam_params;

% -- Add factor to graph
param_ids = [1; 2; 3];
covar = eye(2);
graph.factors{1} = ba_factor(cam_params, param_ids, z);

% Evaluate BA factor
factor = graph.factors{1};

cam_pose = graph.params{factor.param_ids(1)};
lm = graph.params{factor.param_ids(2)};
cam_params = graph.params{factor.param_ids(3)};

% [r, jacobians] = ba_factor_eval(factor, {cam_pose, lm, cam_params})
