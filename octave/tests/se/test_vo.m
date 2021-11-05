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
graph = graph_init();

% -- Add features
printf("Add features\n");
fid2pid = {};
for i = 1:rows(sim_data.features)
  p_W = sim_data.features(i, :)';
  [graph, param_id] = graph_add_param(graph, feature_init(i, p_W));
  fid2pid{i} = param_id;
endfor

% -- Add cam0
printf("Add camera\n");
[graph, cam_id] = graph_add_param(graph, sim_data.cam0);

% Loop through time
printf("Loop through time\n");
for k = 2:length(sim_data.timeline)
  printf(".");
  event = sim_data.timeline(k);

  % Add camera pose
  ts = event.ts;
  T_WC_k = event.cam_pose;
  pose = pose_init(ts, T_WC_k);
  [graph, pose_id] = graph_add_param(graph, pose);

  % Add camera factor
  for i = 1:length(event.cam_p_data)
    fid = event.cam_p_data(i);
    feature_id = fid2pid{fid};
    z = event.cam_z_data(:, i);

    param_ids = [pose_id, feature_id, cam_id];
    covar = eye(2);
    ba_factor = ba_factor_init(0, param_ids, z, covar);
    graph = graph_add_factor(graph, ba_factor);
  endfor
endfor
printf("\n");

graph = graph_solve(graph);
