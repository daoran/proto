addpath(genpath("proto"));

function camera_pose = setup_camera_pose()
  rot = euler2quat(deg2rad([-90; 0; -90]));
  trans = [0.01; 0.02; 0.03];
  T_WC = tf(rot, trans);
  data = tf_param(T_WC);
  camera_pose = pose_init(0, data);
endfunction

function camera = setup_camera()
  cam_idx = 0;
  image_width = 640;
  image_height = 480;
  resolution = [image_width; image_height];
  fov = 60.0;
  fx = focal_length(image_width, fov);
  fy = focal_length(image_height, fov);
  cx = image_width / 2;
  cy = image_height / 2;
  proj_params = [fx; fy; cx; cy];
  dist_params = [-0.01; 0.01; 1e-4; 1e-4];
  camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);
endfunction

function [features, measurements] = setup_features(camera, camera_pose)
  nb_features = 10;

  features = {};
  measurements = {};
  T_WC = tf(camera_pose.param);

  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);

  for i = 1:nb_features
    p_W = [10; rand(2, 1)];
    p_C = tf_point(inv(T_WC), p_W);
    z = camera.project(proj_params, dist_params, p_C);
    feature = feature_init(0, p_W + rand(3, 1) * 0.01);

    features{i} = feature;
    measurements{i} = z;
  endfor
endfunction

function [r, H, g, param_indices] = f(camera_pose, camera, features, measurements)
  % Setup graph
  graph = graph_init();

  for i = 1:length(features)
    % Add params
    [graph, camera_pose_id] = graph_add_param(graph, camera_pose);
    [graph, feature_id] = graph_add_param(graph, features{i});
    [graph, cam_params_id] = graph_add_param(graph, camera);

    % Create factor
    ts = 0;
    param_ids = [camera_pose_id; feature_id; cam_params_id];
    ba_factor = ba_factor_init(ts, param_ids, measurements{i});

    % Add factor and evaluate
    graph = graph_add_factor(graph, ba_factor);
  endfor

  [H, g, r, param_indices] = graph_eval(graph);
endfunction

camera = setup_camera();
camera_pose = setup_camera_pose();
[features, measurements] = setup_features(camera, camera_pose);

% Update without marginalization
[r, H, g, param_indices] = f(camera_pose, camera, features, measurements);
printf("\n");
printf("[before optimizing] reproj_error: %f px\n", norm(r));

lambda = 1e-4;
H = H + lambda * eye(size(H));
dx = H \ g;

camera_pose = pose_update(camera_pose, dx(1:6));
for i = 0:length(features)-1
  dx_start = i * 3 + 1 + 6;
  dx_end = i * 3 + 1 + 2 + 6;
  features{i+1}.param += dx(dx_start:dx_end);
endfor
camera.param += dx(end-7:end);

imagesc(dx);
ginput();

[r, H, g, param_indices] = f(camera_pose, camera, features, measurements);
printf("[after optimizing]  reproj_error: %f px\n", norm(r));

% % Update with marginalization
% [r, H, g, param_indices] = f(camera_pose, camera, features, measurements);
% printf("\n");
% printf("[before optimizing] reproj_error: %f px\n", norm(r));
%
% m = 6;
% r = rows(H) - m;
% [H_marg, g_marg] = schurs_complement(H, g, m, r);
%
% lambda = 1e-4;
% H_marg = H_marg + lambda * eye(size(H_marg));
% dx = H_marg \ g_marg;
%
% for i = 0:length(features)-1
%   dx_start = i * 3 + 1
%   dx_end = i * 3 + 1 + 2
%   features{i+1}.param += dx(dx_start:dx_end);
% endfor
% camera.param += dx(end-7:end);
%
% [r, H, g] = f(camera_pose, camera, features, measurements);
% printf("[after optimizing]  reproj_error: %f px\n", norm(r));
