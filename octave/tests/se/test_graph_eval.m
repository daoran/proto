addpath(genpath("proto"));
graphics_toolkit("fltk");

% Simulate calibration data
% -- Create calibration target
calib_target = calib_target_init(5, 5);
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [1.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);
% -- Create camera
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
camera = camera_init(cam_idx, resolution,
                     proj_model, dist_model,
                     proj_params, dist_params);
% -- Simulate
nb_poses = 10;
data_gnd = calib_sim(calib_target, T_WT, camera, nb_poses);
data = calib_data_add_noise(data_gnd);

% Create graph
graph = graph_init();
% -- Add camera
[graph, camera_id] = graph_add_param(graph, camera);
% -- Add landmarks
lmid2pid = {}; % Landmark id to parameter id
for i = 1:columns(data.p_data)
	p_W = data.p_data(:, i);
	[graph, param_id] = graph_add_param(graph, landmark_init(i, p_W));
	lmid2pid{i} = param_id;
endfor
% -- Loop through time
for k = 1:length(data.time)
	% Timestamp
	ts = data.time(k);

	% Form camera pose transform T_WC
	q_WC = data.q_WC{k};
	r_WC = data.r_WC{k};
	cam_pose = pose_init(ts, [q_WC; r_WC]);
	[graph, pose_id] = graph_add_param(graph, cam_pose);

	% Get point ids and keypoint measurements at time k
	p_ids = data.point_ids_data{k};
	z_k = data.z_data{k};

	% Loop over observations at time k
	for i = 1:length(p_ids)
		p_W = data.p_data(:, p_ids(i));
		landmark_id = lmid2pid{p_ids(i)};
		ba_factor = ba_factor_init(0, [pose_id, landmark_id, camera_id], z_k(:, i));
		graph = graph_add_factor(graph, ba_factor);
	endfor
endfor

% Optimize
for i = 1:10
	[H, g, r, param_idx] = graph_eval(graph);
	H = H + 0.1 * eye(size(H)); % Levenberg-Marquardt Dampening
	dx = pinv(H) * g;

	graph = graph_update(graph, param_idx, dx);
	cost = 0.5 * r' * r
end
