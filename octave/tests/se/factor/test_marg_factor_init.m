addpath(genpath("proto"));

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
cam_params = camera_init(cam_idx, resolution,
                         proj_model, dist_model,
                         proj_params, dist_params);

% Setup graph
graph = graph_init();

% -- Create BA factors
for i = 1:5
  % Camera pose T_WC
  rot = euler2quat(deg2rad([-90; 0; -90]));
  trans = [0.0; 0.0; 0.0];
  T_WC = tf(rot, trans);
  data = [rot; trans];
  cam_pose = pose_init(0, data);

  % Feature
  feature = feature_init(0, [10; rand(2, 1)]);

  % Add parameters to graph
  [graph, cam_pose_id] = graph_add_param(graph, cam_pose);
  [graph, feature_id] = graph_add_param(graph, feature);
  [graph, cam_params_id] = graph_add_param(graph, cam_params);

  % BA Factor
  p_C = tf_point(inv(T_WC), feature.param);
  z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C);
  param_ids = [cam_pose_id; feature_id; cam_params_id];
  ba_factor = ba_factor_init(i, param_ids, z);
  graph = graph_add_factor(graph, ba_factor);
endfor

marg_factors = {};
marg_param_indicies = {};
marg_factors{1} = graph.factors{1};
marg_param_indicies{1} = [2];

marg_factor = marg_factor_init(marg_factors, marg_param_indicies);

params = graph_get_params(graph, marg_factor.param_ids);
[r, jacs] = marg_factor_eval(marg_factor, params);

% params = [];
% marg_factor_eval(marg_factor, params);
