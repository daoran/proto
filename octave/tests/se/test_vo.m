addpath(genpath("proto"));

% Simulate VO data
save_path = "/tmp/test_vo.data";
if length(glob(save_path)) == 0
  sim_data = sim_vo(2.0, 10.0);
  save("-binary", save_path, "sim_data");
else
  load(save_path);
endif

% Ground truth data
gnd_data = {};
gnd_data.cam_pos = [sim_data.cam_pos(1)];
gnd_data.cam_att = [sim_data.cam_att(1)];
gnd_data.p_data = sim_data.cam_p_data;

% Create graph
printf("Create factor graph\n");
inv_depth_mode = true;
graph = graph_init();

% Add cam0
printf("Add camera\n");
camera = sim_data.cam0;
[graph, cam_id] = graph_add_param(graph, camera);

% Loop through time
printf("Loop through time\n");

% Initialize features container
cam_features = {};
for i = 1:rows(sim_data.features)
  cam_features{i} = 0;
endfor

% Initialize first pose
event = sim_data.timeline(1);
% -- Add camera pose
ts = event.ts;
T_WC_k = event.cam_pose;
pose = pose_init(ts, T_WC_k);
[graph, pose_id] = graph_add_param(graph, pose);
% -- Add bundle adjustment factors
for i = 1:length(event.cam_p_data)
  feature_idx = event.cam_p_data(i);
  z = event.cam_z_data(:, i);

  % Add feature - inverse depth parameterization
  if cam_features{feature_idx} == 0
    feature_param = 0;
    if inv_depth_mode
      feature_param = idp_param(camera, T_WC_k, z);
    else
      feature_param = sim_data.features(feature_idx, :)';
    endif

    feature = feature_init(feature_idx, feature_param);
    [graph, param_id] = graph_add_param(graph, feature);
    cam_features{feature_idx} = param_id;
  endif

  % Add factor
  feature_id = cam_features{feature_idx};
  param_ids = [pose_id, feature_id, cam_id];
  covar = eye(2);
  ba_factor = ba_factor_init(0, param_ids, z, covar);
  graph = graph_add_factor(graph, ba_factor);
endfor

% -- Loop
fig = figure(1);
hold on;
draw_frame(T_WC_k);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
axis('equal')
view(3);

T_WC_km1 = T_WC_k;
for k = 2:length(sim_data.timeline)
  % Event
  printf("frame_idx: %d\n", k);
  event = sim_data.timeline(k);

  % Add camera pose
  ts = event.ts;
  pose = pose_init(ts, T_WC_km1);
  [graph, pose_id] = graph_add_param(graph, pose);

  % Use already tracking features to estimate current T_WC_k
  for i = 1:length(event.cam_p_data)
    feature_idx = event.cam_p_data(i);
    if cam_features{feature_idx}
      feature_id = cam_features{feature_idx};
      param_ids = [pose_id, feature_id, cam_id];
      z = event.cam_z_data(:, i);
      covar = eye(2);
      ba_factor = ba_factor_init(ts, param_ids, z, covar);
      graph = graph_add_factor(graph, ba_factor);
    endif
  endfor
  graph = graph_solve(graph, 5);
  printf("\n");
  % break;

  % Add new features
  pose_est = graph.params{graph.pose_param_ids(end)};
  T_WC_k = tf(pose_est.param);

  for i = 1:length(event.cam_p_data)
    feature_idx = event.cam_p_data(i);
    z = event.cam_z_data(:, i);

    if cam_features{feature_idx} == 0
      feature_param = 0;
      if inv_depth_mode
        feature_param = idp_param(camera, T_WC_k, z);
      else
        feature_param = sim_data.features(feature_idx, :)';
      endif
      feature = feature_init(feature_idx, feature_param);
      [graph, param_id] = graph_add_param(graph, feature);
      cam_features{feature_idx} = param_id;
    endif
  endfor

  % % Compare estimated features against ground truth
  % features_diff = [];
  % points = [];
  % for i = 1:length(cam_features)
  %   if cam_features{i} != 0
  %     % Ground truth feature position
  %     p_W_gnd = sim_data.features(i, :)';
  %
  %     % Estimated feature position
  %     param_id = cam_features{i};
  %     feature = graph.params{param_id};
  %     p_W_est = zeros(3, 1);
  %     if strcmp(feature.parameterization, "XYZ")
  %       p_W_est = feature.param;
  %     elseif strcmp(feature.parameterization, "INVERSE_DEPTH")
  %       p_W_est = idp_point(feature.param);
  %     endif
  %     points = [points, p_W_est];
  %
  %     % Compare
  %     features_diff = [features_diff, norm(p_W_gnd - p_W_est)];
  %     points = [points, p_W_est];
  %   endif
  % endfor
  % scatter3(points(1, :), points(2, :), points(3, :));
  % printf("points diff: %f\n", mean(features_diff));
  % printf("\n");

  % Update
  draw_frame(T_WC_k);
  refresh(fig);
  T_WC_km1 = T_WC_k;
endfor
