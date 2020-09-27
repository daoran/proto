addpath(genpath("proto"));
graphics_toolkit("fltk");
pkg load statistics;
% profile on;

function data = calib_data_add_noise(data)
  nb_poses = length(data.time);

  % Add noise to camera position and rotation
  for i = 1:nb_poses
    % Position
    data.r_WC{i} += normrnd([0; 0; 0], 1e-2);

    % Rotation
    dq = quat_delta(normrnd([0; 0; 0], 1e-1));
    q_WC = data.q_WC{i};
    data.q_WC{i} = quat_mul(dq, q_WC);

    % Keypoint measurements
    data.z_data{i} += normrnd(zeros(2, length(data.z_data{i})), 1e-2);
  endfor

  % Add noise to point data
  data.p_data += normrnd(zeros(3, length(data.p_data)), 1e-2);
endfunction

function retval = ba_nb_poses(data)
  retval = length(data.time);
endfunction

function retval = ba_nb_measurements(data)
  nb_poses = ba_nb_poses(data);
  retval = 0;
  for i = 1:nb_poses
    retval += length(data.z_data{i});
  endfor
endfunction

function J = intrinsics_point_jacobian(camera)
  proj_params = camera.param(1:4);
  fx = proj_params(1);
  fy = proj_params(2);

  J = zeros(2, 2);
  J(1, 1) = fx;
  J(2, 2) = fy;
endfunction

function J = project_jacobian(p_C)
  x = p_C(1);
  y = p_C(2);
  z = p_C(3);

  J = zeros(2, 3);
  J(1, 1) = 1.0 / z;
  J(2, 2) = 1.0 / z;
  J(1, 3) = -x / z**2;
  J(2, 3) = -y / z**2;
endfunction

function retval = check_J_cam_rot(K, T_WC, p_W, J_cam_rot, step_size, threshold)
  fdiff = zeros(2, 3);
  z = [0; 0];
  e = z - pinhole_project(K, T_WC, p_W);

  for i = 1:3
    T_WC_diff = perturb_rot(T_WC, step_size, i);
    e_prime = z - pinhole_project(K, T_WC_diff, p_W);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("J_cam_rot", fdiff, J_cam_rot, threshold);
endfunction

function retval = check_J_cam_pos(K, T_WC, p_W, J_cam_pos, step_size, threshold)
  fdiff = zeros(2, 3);
  z = [0; 0];
  e = z - pinhole_project(K, T_WC, p_W);

  for i = 1:3
    T_WC_diff = perturb_trans(T_WC, step_size, i);
    e_prime = z - pinhole_project(K, T_WC_diff, p_W);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("J_cam_pos", fdiff, J_cam_pos, threshold);
endfunction

function retval = check_J_point(K, T_WC, p_W, J_point, step_size, threshold)
  fdiff = zeros(2, 3);
  dr = eye(3) * step_size;
  z = [0; 0];
  e = z - pinhole_project(K, T_WC, p_W);

  for i = 1:3
    p_W_diff = p_W + dr(1:3, i);
    e_prime = z - pinhole_project(K, T_WC, p_W_diff);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("J_point", fdiff, J_point, threshold);
endfunction

function J = camera_rotation_jacobian(q_WC, r_WC, p_W)
  C_WC = quat2rot(q_WC);
  J = C_WC' * skew(p_W - r_WC);
endfunction

function J = camera_translation_jacobian(q_WC)
  C_WC = quat2rot(q_WC);
  J = -C_WC';
endfunction

function J = target_point_jacobian(q_WC)
  C_WC = quat2rot(q_WC);
  J = C_WC';
endfunction

function J = ba_jacobian(data, check_jacobians=false)
  nb_poses = ba_nb_poses(data);
  nb_points = data.target.nb_rows * data.target.nb_cols;
  nb_measurements = ba_nb_measurements(data);

  % Setup jacobian
  J_rows = nb_measurements * 2;
  J_cols = (nb_poses * 6) + (nb_points * 3);
  J = zeros(J_rows, J_cols);

  % Loop over camera poses
  pose_idx = 1;
  meas_idx = 1;
  for i = 1:nb_poses
    % Form camera pose transform T_WC
    q_WC = data.q_WC{i};
    r_WC = data.r_WC{i};
    T_WC = tf(q_WC, r_WC);

    % Invert T_WC to T_CW and decompose
    T_CW = tf_inv(T_WC);
    C_CW = tf_rot(T_CW);
    r_CW = tf_trans(T_CW);

    # Get point ids and keypoint measurements at time k
    p_ids = data.point_ids_data{i};
    z_k = data.z_data{i};

    % Loop over observations at time k
    for j = 1:length(p_ids)
      % Transform point from world to camera frame
      p_W = data.p_data(:, p_ids(j));
      p_C = dehomogeneous(T_CW * homogeneous(p_W));

      % Fill in camera pose jacobian
      % -- Setup row start, row end, column start and column end
      rs = ((meas_idx - 1) * 2) + 1;
      re = rs + 1;
      cs = ((pose_idx - 1) * 6) + 1;
      ce = cs + 5;
      % -- Form jacobians
      J_K = intrinsics_point_jacobian(data.camera);
      J_P = project_jacobian(p_C);
      J_cam_rot = -1 * J_K * J_P * camera_rotation_jacobian(q_WC, r_WC, p_W);
      J_cam_pos = -1 * J_K * J_P * camera_translation_jacobian(q_WC);
      % -- Fill in the big jacobian
      J(rs:re, cs:ce) = [J_cam_rot, J_cam_pos];
      % J(rs:re, cs:ce) = 255 * ones(2, 6);

      % Fill in point elements
      % -- Setup row start, row end, column start and column end
      cs = (nb_poses * 6) + ((p_ids(j) - 1) * 3) + 1;
      ce = cs + 2;
      % -- Fill in the big jacobian
      J_point = -1 * J_K * J_P * target_point_jacobian(q_WC);
      J(rs:re, cs:ce) = J_point;
      % J(rs:re, cs:ce) = 255 * ones(2, 3);

      % Test jacobians
      if check_jacobians
        step_size = 1.0e-8;
        threshold = 1.0e-4;
        check_J_cam_rot(K, T_WC, p_W, J_cam_rot, step_size, threshold);
        check_J_cam_pos(K, T_WC, p_W, J_cam_pos, step_size, threshold);
        check_J_point(K, T_WC, p_W, J_point, step_size, threshold);
      endif

      meas_idx++;
    endfor

    pose_idx++;
  endfor

  % figure();
  % cmap = colormap();
  % imshow(J, cmap);
  % ginput();
endfunction

function residuals = ba_residuals(data)
  residuals = [];

  % Target pose
  C_WT = quat2rot(data.q_WT);
  r_WT = data.r_WT;
  T_WT = tf(C_WT, r_WT);

  % Loop over time
  for k = 1:length(data.time)
    % Form camera pose
    q_WC = data.q_WC{k};
    r_WC = data.r_WC{k};
    T_WC = tf(q_WC, r_WC);

    % Get point ids and measurements at time step k
    point_ids = data.point_ids_data{k};
    z_k = data.z_data{k};

    % Loop over observations at time k
    for i = 1:length(z_k)
      % Form calibration target point in world frame
      p_id = point_ids(i);
      p_W = data.p_data(:, p_id);

      % Calculate reprojection error
      z_hat = pinhole_project(data.camera.param(1:4), T_WC, p_W);
      e = z_k(1:2, i) - z_hat;
      residuals = [residuals; e];
    endfor
  endfor
endfunction

function cost = ba_cost(e)
  cost = 0.5 * e' * e;
endfunction

function [data, EWE] = ba_update(data, e, E, sigma=[1.0; 1.0])
  assert(size(sigma) == [2, 1]);

  % Form weight matrix
  nb_measurements = ba_nb_measurements(data);
  W = diag(repmat(sigma, nb_measurements, 1));

  % Calculate update
  EWE = (E' * W * E);
  dx = pinv(EWE) * (-E' * W * e);

  % EWE = EWE + 0.1 * eye(size(EWE));
  % dx = EWE \ (-E' * e);

  % dx = (EWE) \ (-E' * W * e);
  % dx = inv(EWE) * (-E' * W * e);
  % dx = (EWE)^-1 * (-E' * W * e);

  % Update camera poses
  nb_poses = length(data.time);
  for i = 1:nb_poses
    s = ((i - 1) * 6) + 1;
    dalpha = dx(s:s+2);
    dr_WC = dx(s+3:s+5);

    % Update camera rotation
    dq = quat_delta(dalpha);
    data.q_WC{i} = quat_mul(dq, data.q_WC{i});

    % Update camera position
    data.r_WC{i} += dr_WC;
  endfor

  % Update points
  for i = 1:length(data.p_data)
    s = (nb_poses * 6) + ((i - 1) * 3) + 1;
    dp_W = dx(s:s+2);
    data.p_data(1:3, i) += dp_W;
  endfor
endfunction

function [entropy_data, data, data_gnd] = ba_nbv_solve(data, data_gnd, nb_nbv=0)
  % Setup
  % plot_compare_data(data_gnd, data);
  entropy_data = [];

  % Slice ground truth data
  gnd_end = length(data.time);
  data_gnd.time = data_gnd.time(1:gnd_end);
  data_gnd.camera = data_gnd.camera;
  data_gnd.target = data_gnd.target;
  data_gnd.q_WT = data_gnd.q_WT;
  data_gnd.r_WT = data_gnd.r_WT;
  data_gnd.q_WC = data_gnd.q_WC(1:gnd_end);
  data_gnd.r_WC = data_gnd.r_WC(1:gnd_end);
  data_gnd.z_data = data_gnd.z_data(1:gnd_end);
  data_gnd.point_ids_data = data_gnd.point_ids_data(1:gnd_end);
  data_gnd.p_data = data_gnd.p_data;

  for i = 1:nb_nbv
    nb_poses = length(data.time)

    max_iter = 20;
    cost_prev = 0.0;
    for i = 1:max_iter
      E = ba_jacobian(data);
      e = ba_residuals(data);
      [data, EWE] = ba_update(data, e, E);
      cost = ba_cost(e);
      printf("iter: %d \t\t cost: %.4e\n", i, cost);

      % Termination criteria
      cost_diff = abs(cost - cost_prev);
      if cost_diff < 1e-6
        break;
      endif
      cost_prev = cost;
    endfor

    % Record entropy
    entropy_data = [entropy_data, ba_entropy(data)];

    % Create NBV and add to data
    printf("\n");
    nbv = ba_nbv(data);

    idx_new = length(data.time) + 1;
    data_gnd.time = [data.time, idx_new];
    data_gnd.r_WC{idx_new} = nbv.r_WC;
    data_gnd.q_WC{idx_new} = nbv.q_WC;
    data_gnd.z_data{idx_new} = nbv.z;
    data_gnd.point_ids_data{idx_new} = nbv.point_ids;

    % -- Add noise to position
    nbv.r_WC += normrnd([0; 0; 0], 1e-2);
    % -- Add noise to rotation
    dq = quat_delta(normrnd([0; 0; 0], 1e-1));
    nbv.q_WC = quat_mul(dq, nbv.q_WC);
    % -- Add noise to measurement
    nbv.z += normrnd(zeros(2, length(nbv.z)), 1e-2);

    data.time = [data.time, idx_new];
    data.r_WC{idx_new} = nbv.r_WC;
    data.q_WC{idx_new} = nbv.q_WC;
    data.z_data{idx_new} = nbv.z;
    data.point_ids_data{idx_new} = nbv.point_ids;

    assert(length(nbv.point_ids) == columns(nbv.z));
    printf("Total poses: %d\n", length(data.time));
    printf("Add new NBV\n\n");
  endfor

  % Record last entropy
  max_iter = 20;
  cost_prev = 0.0;
  for i = 1:max_iter
    E = ba_jacobian(data);
    e = ba_residuals(data);
    [data, EWE] = ba_update(data, e, E);
    cost = ba_cost(e);
    printf("iter: %d \t\t cost: %.4e\n", i, cost);

    % Termination criteria
    cost_diff = abs(cost - cost_prev);
    if cost_diff < 1e-6
      break;
    endif
    cost_prev = cost;
  endfor
  entropy_data = [entropy_data, ba_entropy(data)];

  % plot_compare_data(data_gnd, data);
  % ginput();
endfunction

function [entropy_data, data] = ba_batch_solve(data)
  entropy_data = [];
  nb_poses = length(data.time);

  % Optimize
  for k = 1:nb_poses
    printf("\n");
    printf("k: %d\n", k);
    opt_data.time = data.time(1:k);
    opt_data.camera = data.camera;
    opt_data.target = data.target;
    opt_data.q_WT = data.q_WT;
    opt_data.r_WT = data.r_WT;
    opt_data.q_WC = data.q_WC(1:k);
    opt_data.r_WC = data.r_WC(1:k);
    opt_data.z_data = data.z_data(1:k);
    opt_data.point_ids_data = data.point_ids_data(1:k);
    opt_data.p_data = data.p_data;

    max_iter = 20;
    cost_prev = 0.0;
    for i = 1:max_iter
      E = ba_jacobian(opt_data);
      e = ba_residuals(opt_data);
      [opt_data, EWE] = ba_update(opt_data, e, E);
      cost = ba_cost(e);
      printf("iter: %d \t\t cost: %.4e\n", i, cost);

      % Termination criteria
      cost_diff = abs(cost - cost_prev);
      if cost_diff < 1e-6
        break;
      endif
      cost_prev = cost;
    endfor

    % Record entropy
    entropy_data = [entropy_data, ba_entropy(opt_data)];
  endfor

  % plot_compare_data(data_gnd, data);
  % ginput();
endfunction

function entropy = ba_entropy(data, sigma=[1.0; 1.0])
  % Form weight matrix
  nb_measurements = ba_nb_measurements(data);
  W = diag(repmat(sigma, nb_measurements, 1));

  % Calculate update
  E = ba_jacobian(data);
  H = (E' * W * E);
  % lambda = 1e-10;
  % H = H + (lambda * eye(size(H)));

  % Estimate covariance
  % covar = (H)^-1;
  % rank(H)
  % rows(H)
  covar = pinv(H);
  nb_points = length(data.p_data);
  points_covar = covar(end-(nb_points*3)+1:end, end-(nb_points*3)+1:end);

  % Calculate Shannon Entropy
  % n = rows(covar);
  % entropy = 0.5 * log((2 * pi * e)**n * det(covar));
  n = length(points_covar);
  entropy = 0.5 * log((2 * pi * e)^n * det(points_covar));
endfunction

function nbv = ba_nbv(data, sigma=[1.0; 1.0])
  printf("Calculating NBV\n");
  % Genereate candidate poses
  nb_poses = 100;
  [poses, calib_center] = calib_generate_random_poses(data.target, nb_poses);
  % [poses, calib_center] = calib_generate_poses(data.target);

  % Setup
  entropy_init = ba_entropy(data, sigma);
  idx_new = length(data.time) + 1;
  entropy_min = entropy_init;
  nbv = {};

  % Evaluate entropy of candidate poses
  for i = 1:length(poses)
    T_TC = poses{i};
    T_WT = tf(data.q_WT, data.r_WT);
    T_WC = T_WT * T_TC;
    r_WC = tf_trans(T_WC);
    q_WC = tf_quat(T_WC);

    p_data = data.p_data;
    [z, point_ids] = camera_measurements(data.camera, T_WC, p_data);

    data_new = data;
    data_new.time = [data_new.time, idx_new];
    data_new.r_WC{idx_new} = r_WC;
    data_new.q_WC{idx_new} = q_WC;
    data_new.z_data{idx_new} = z;
    data_new.point_ids_data{idx_new} = point_ids;

    entropy = ba_entropy(data_new);
    if entropy < entropy_min
      entropy_min = entropy;
      nbv.r_WC = data_new.r_WC{idx_new};
      nbv.q_WC = data_new.q_WC{idx_new};
      nbv.z = data_new.z_data{idx_new};
      nbv.point_ids = data_new.point_ids_data{idx_new};
      assert(length(nbv.point_ids) == columns(nbv.z));
    endif
  endfor

  printf("Initial entropy: %f\n", entropy_init);
  printf("Final pose candidate entropy: %f\n", entropy_min);
endfunction

function plot_data(data)
  nb_poses = length(data.time);
  q_WT = data.q_WT;
  r_WT = data.r_WT;
  T_WT = tf(q_WT, r_WT);

  for i = 1:nb_poses
    q_WC = data.q_WC{i};
    r_WC = data.r_WC{i};
    T_WC = tf(q_WC, r_WC);

    calib_target_draw(data.target, T_WT);
    draw_camera(T_WC);
    draw_frame(T_WC, 0.05);
  endfor
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  view(3);
  axis 'equal';
endfunction

function plot_compare_data(data_gnd, data_est)
  nb_poses = length(data_gnd.time);
  q_WT = data_gnd.q_WT;
  r_WT = data_gnd.r_WT;
  T_WT = tf(q_WT, r_WT);

  % Figure
  figure();
  clf();
  hold on;

  % -- Plot ground truth data
  for i = 1:nb_poses
    q_WC = data_gnd.q_WC{i};
    r_WC = data_gnd.r_WC{i};
    T_WC = tf(q_WC, r_WC);

    calib_target_draw(data_gnd.target, T_WT);
    draw_camera(T_WC, 0.05, "r-");
    % draw_frame(T_WC, 0.05);
  endfor

  % -- Plot estimated data
  for i = 1:nb_poses
    q_WC = data_est.q_WC{i};
    r_WC = data_est.r_WC{i};
    T_WC = tf(q_WC, r_WC);

    draw_camera(T_WC, 0.05, "b-");
    % draw_frame(T_WC, 0.05);
  endfor

  scatter3(data_est.p_data(1, :),
           data_est.p_data(2, :),
            data_est.p_data(3, :),
           10.0,
            'b');

  % Plot settings
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  view(3);
  axis 'equal';
endfunction


% Setup data
% -- Create calibration target
calib_target = calib_target_init(4, 4);
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [1.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);
% -- Create camera
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 90.0;
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
% -- Create data
% data = trajectory_simulate(camera, chessboard);
nb_poses = 10;
data_gnd = calib_sim(calib_target, T_WT, camera, nb_poses);
data = calib_data_add_noise(data_gnd);

% Batch optimization
[batch_entropy, batch_data] = ba_batch_solve(data, 3);

% NBV optimization
nbv_data.time = data.time(1);
nbv_data.camera = data.camera;
nbv_data.target = data.target;
nbv_data.q_WT = data.q_WT;
nbv_data.r_WT = data.r_WT;
nbv_data.q_WC = data.q_WC(1);
nbv_data.r_WC = data.r_WC(1);
nbv_data.z_data = data.z_data(1);
nbv_data.point_ids_data = data.point_ids_data(1);
nbv_data.p_data = data.p_data;

nb_nbvs = 9;
[nbv_entropy, nbv_data, nbv_gnd] = ba_nbv_solve(nbv_data, data_gnd, nb_nbvs);


% % Plot
% plot_compare_data(data_gnd, batch_data);
% title("Batch");

% plot_compare_data(nbv_gnd, nbv_data);
% title("NBV");
% ginput();

size(batch_entropy)
size(nbv_entropy)

figure();
hold on;
x_axis = (nbv_data.time(end)-nb_nbvs:nbv_data.time(end));
plot(x_axis, batch_entropy, "r-", "linewidth", 2.0);
plot(x_axis, nbv_entropy, "b-", "linewidth", 2.0);
legend("Batch", "NBV");
set(gca, "XTick", x_axis)
xlabel("Number of views");
ylabel("Covariance Entropy [nats]");
ginput();
