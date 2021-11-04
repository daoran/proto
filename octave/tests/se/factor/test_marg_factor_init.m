addpath(genpath("proto"));

function camera_pose = setup_camera_pose()
  rot = euler2quat(deg2rad([-90; 0; -90]));
  trans = [0.01; 0.02; 0.03] + rand(3, 1) * 0.01;
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
  camera.fixed = true;
endfunction

function feature_data = setup_features(nb_features)
  feature_data = {};
  feature_data.points = {};
  feature_data.features = {};

  for i = 1:nb_features
    p_W = [10; rand(2, 1)];
    feature_data.points{i} = p_W;
    feature_data.features{i} = feature_init(i, p_W + rand(3, 1) * 0.1);
  endfor
endfunction

function camera_data = setup_data(camera, poses, feature_data, observations)
  assert(length(poses) == length(observations));

  % Setup
  nb_poses = length(poses);
  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);

  % Create camera views
  camera_data = {};
  camera_data.camera = camera;
  camera_data.points = feature_data.points;
  camera_data.features = feature_data.features;
  camera_data.poses = {};
  camera_data.views = {};

  for k = 1:nb_poses
    T_WC = tf(poses{k}.param);
    T_CW = inv(T_WC);
    camera_data.poses{k} = pose_init(k, T_WC);

    camera_view = {};
    camera_view.feature_indices = [];
    camera_view.measurements = {};

    for i = 1:length(observations{k})
      feature_idx = observations{k}(i);

      p_W = camera_data.points{feature_idx};
      p_C = tf_point(T_CW, p_W);
      z = camera.project(proj_params, dist_params, p_C);

      camera_view.feature_indices = [camera_view.feature_indices, feature_idx];
      camera_view.measurements{i} = z;
    endfor

    camera_data.views{k} = camera_view;
  endfor
endfunction

function graph = test_setup()
  % Setup Camera
  camera = setup_camera();

  % Setup poses
  poses = {};
  poses{1} = setup_camera_pose();
  poses{2} = setup_camera_pose();
  poses{3} = setup_camera_pose();
  poses{4} = setup_camera_pose();

  % Setup features
  nb_features = 10;
  feature_data = setup_features(nb_features);

  % Setup camera data
  observations = {};
  observations{1} = [1, 2, 3];
  observations{2} = [2, 3, 4];
  observations{3} = [3, 4, 5];
  observations{4} = [4, 5, 6, 7, 8];
  camera_data = setup_data(camera, poses, feature_data, observations);

  % Setup graph
  graph = graph_init();
  % -- Add camera
  camera = camera_data.camera;
  [graph, cam_params_id] = graph_add_param(graph, camera);
  % -- Add features
  features = camera_data.features;
  feature_param_ids = {};
  for idx = 1:length(features)
    [graph, param_id] = graph_add_param(graph, features{idx});
    feature_param_ids{idx} = param_id;
  endfor
  % -- Loop through camera views
  for k = 1:length(camera_data.poses)
    % Add camera pose
    pose = camera_data.poses{k};
    [graph, pose_id] = graph_add_param(graph, pose);

    camera_view = camera_data.views{k};
    for i = 1:length(camera_view.feature_indices)
      % Feature param id
      feature_idx = camera_view.feature_indices(i);
      feature_param_id = feature_param_ids{feature_idx};

      % Create ba factor
      ts = 0;
      param_ids = [pose_id; feature_param_id; cam_params_id];
      z = camera_view.measurements{i};
      ba_factor = ba_factor_init(ts, param_ids, z);

      % Add factor and evaluate
      graph = graph_add_factor(graph, ba_factor);
    endfor
  endfor
endfunction

% Test
graph = test_setup();

marg_param_ids = [graph.pose_param_ids(1)];
marg_factor = marg_factor_init(graph, marg_param_ids);

params = graph_get_params(graph, marg_factor.param_ids);
[r, jacs] = marg_factor_eval(marg_factor, params);
