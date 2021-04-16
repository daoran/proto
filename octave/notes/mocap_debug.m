graphics_toolkit("fltk");
data_path = "/data/aabm/calib_mocap"

function [ts_int] = str2uint64(ts_str)
  assert(typeinfo(ts_str) == "sq_string");

  ts_str = strsplit(ts_str, "."){1};
  ts_int = uint64(0);
  for i = 0:(length(ts_str) - 1)
    ts_int += str2num(ts_str(end - i)) * 10**i;
  end
endfunction

function result = list_dir(target_dir)
  listing = dir(target_dir);
  result = [];

  for i = 1:length(listing)
    if any(strcmp(listing(i).name, {'.', '..'})) == 0
      target.name = listing(i).name;
      target.date = listing(i).date;
      target.bytes = listing(i).bytes;
      target.isdir = listing(i).isdir;

      result = [result; target];
    end
  end
endfunction

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  # Homogeneous form
  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function T = tf(varargin)
  rot = eye(3);
  trans = zeros(3, 1);

  % Parse arguments
  assert(length(varargin) == 1 || length(varargin) == 2);
  if length(varargin) == 1
    pose = varargin{1};
    assert(all(size(pose) == [7, 1]));
    rot = quat2rot(pose(1:4));
    trans = pose(5:7);

  elseif length(varargin) == 2
    rot = varargin{1};
    trans = varargin{2};
    assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
    assert(size(trans) == [3, 1]);
    if size(rot) == [4, 1]
      rot = quat2rot(rot);
    endif

  endif

  T = eye(4, 4);
  T(1:3, 1:3) = rot;
  T(1:3, 4) = trans;
endfunction

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function pose = load_pose(csv_path)
  data = csvread(csv_path, 1, 0);
  q = data(1, 2:5);
  r = data(1, 6:8)';
  pose = tf(quat2rot(q), r);
endfunction

function [ts, poses] = load_poses(csv_path)
  ts = [];
  poses = {};
  data = csvread(csv_path, 1, 0);
  for i = 1:rows(data)
    ts = [ts; data(i, 1)];
    q = data(i, 2:5);
    r = data(i, 6:8)';
    poses{i} = tf(quat2rot(q), r);
  end
endfunction

function camera_ts = load_camera_timestamps(csv_path)
  camera_ts = [];

  % Parse file
  fid = fopen(csv_path, "r");
  % -- Check number of lines
  nb_lines = fskipl(fid, Inf);
  if nb_lines < 2
    return;
  endif
  % -- Parse file
  frewind(fid);
  fmt_str = "%s %s";
  csv_data = textscan(fid, fmt_str, "delimiter", ",", "headerlines", 1);
  fclose(fid);

  % Extract timestamp data
  for i = 1:length(csv_data{1})
    camera_ts = [camera_ts; str2uint64(csv_data{1}{i})];
  endfor
endfunction

function image_paths = load_camera_data(csv_path)
  [ts, image_paths] = textread(csv_path, "%f %s",
                               "delimiter", ",",
                               "headerlines", 1);

  basedir = fileparts(csv_path);
  for i = 1:length(image_paths)
    image_paths(i, 1) = strcat(basedir, "/data/", image_paths{i});
  end
endfunction

function grid_data = load_grid(csv_path)
  % Format string
  fmt_str = "%s";                           % Timestamp
  fmt_str = strcat(fmt_str, "%d %d %f %f"); % Target properties
  fmt_str = strcat(fmt_str, "%f %f");       % Keypoints
  fmt_str = strcat(fmt_str, "%f %f %f");    % Object points
  fmt_str = strcat(fmt_str, "%d");          % Tag id
  fmt_str = strcat(fmt_str, "%d");          % Corner index

  % Parse file
  fid = fopen(csv_path, "r");
  % -- Check number of lines
  nb_lines = fskipl(fid, Inf);
  if nb_lines < 2
    retval = -1;
    grid_data = {};
    return;
  endif
  % -- Parse file
  frewind(fid);
  csv_data = textscan(fid, fmt_str, "delimiter", ",", "headerlines", 1);
  fclose(fid);

  % Timestamp
  grid_data.ts = str2uint64(csv_data{1}{1});

  % Target properties
  grid_data.tag_rows = csv_data{2}(1);
  grid_data.tag_cols = csv_data{3}(1);
  grid_data.tag_size = csv_data{4}(1);
  grid_data.tag_spacing = csv_data{5}(1);

  % Keypoints
  kp_x = csv_data{6}(1:end);
  kp_y = csv_data{7}(1:end);
  grid_data.keypoints = [kp_x, kp_y]';

  % Object points
  pt_x = csv_data{8}(1:end);
  pt_y = csv_data{9}(1:end);
  pt_z = csv_data{10}(1:end);
  grid_data.object_points = [pt_x, pt_y, pt_z]';

  % Tag ids, corner indicies
  grid_data.tag_ids = csv_data{11}(1:end);
  grid_data.corner_indicies = csv_data{12}(1:end);
endfunction

function grids = load_grids(data_dir, cam_idx)
  grids_dir = strcat(data_dir, "/cam", num2str(cam_idx), "/data");
  grid_csv_paths = list_dir(grids_dir);

  grids = {};
  for i = 1:length(grid_csv_paths)
    grid_path = strcat(grids_dir, "/", grid_csv_paths(i).name);
    grids{i} = load_grid(grid_path);
  endfor
endfunction

function [disp_x, disp_y, disp_z] = extract_positions(poses)
  disp_x = [];
  disp_y = [];
  disp_z = [];

  for i = 1:length(poses)
    r = tf_trans(poses{i});
    disp_x = [disp_x; r(1)];
    disp_y = [disp_y; r(2)];
    disp_z = [disp_z; r(3)];
  endfor
endfunction

function [poses_ts, poses_sampled] = sample_poses(ts, poses, sample_ts)
  poses_ts = [];
  poses_sampled = {};

  ts_cand = ts(1);
  pose_cand = 1;
  sample_idx = 1;

  for i = 1:length(poses)
    if ts(i) > sample_ts(sample_idx)
      % Not the most precise but will do for now
      poses_ts = [poses_ts; sample_ts(sample_idx)];
      poses_sampled{sample_idx} = poses{i};
      sample_idx += 1;
    endif

    if sample_idx > length(sample_ts)
      break;
    end
  endfor
endfunction

function plot_pose_capture(marker_ts, marker_poses, camera_ts)
  figure();
  hold on;

  [x, y, z] = extract_positions(marker_poses);
  t = (marker_ts - repmat(marker_ts(1), length(marker_ts), 1)) * 1e-9;
  cam_t = (camera_ts - repmat(marker_ts(1), length(camera_ts), 1)) * 1e-9;

  plot(t, x, 'r-', 'linewidth', 2.0);
  plot(t, y, 'g-', 'linewidth', 2.0);
  plot(t, z, 'b-', 'linewidth', 2.0);

  for i = 1:length(cam_t)
    plot([cam_t(i), cam_t(i)], [-10, 10], 'k-', "linewidth", 2.0);
  endfor

  xlim([min([t; cam_t]), max([t; cam_t])]);
  ylim([min([x; y; z]), max([x; y; z])]);

  xlabel("Time [s]");
  ylabel("Displacement [m]");
  legend_str = {"x", "y", "z", "Camera Capture"};
  legend(legend_str, 'location', 'eastoutside' );

  ginput();
endfunction

function plot_scene(T_WF, marker_ts, marker_poses, camera_ts)
  [marker_ts, marker_poses] = sample_poses(marker_ts, marker_poses, camera_ts);
  T_MC0 = [
    0.080016, -0.909654, 0.407588, 0.096363
    -0.996769, -0.070128, 0.039169, 0.024375
    -0.007047, -0.409405, -0.912326, -0.077259
    0.000000, 0.000000, 0.000000, 1.000000
  ];

  figure(1);
  for i = 1:length(marker_ts)
    clf;
    hold on;
    grid on;
    view(3);

    printf("timestamp: %ld\n", marker_ts(i));
    T_WM = marker_poses{i};
    draw_frame(T_WF, 0.1);
    draw_frame(T_WM, 0.1);
    draw_frame(T_WM * T_MC0, 0.1);

    xlabel("x");
    ylabel("y");
    zlabel("z");
    axis('equal');
    ginput();
  endfor
endfunction

function inspect_coverage(grid_data)
  coverage_x = [];
  coverage_y = [];
  for i = 1:length(grid_data)
    keypoints = grid_data{i}.keypoints;
    coverage_x = [coverage_x; keypoints(1, :)'];
    coverage_y = [coverage_y; keypoints(2, :)'];
  endfor

  figure('name', "Keypoint Coverage in Image Plane");
  hold on;
  plot(coverage_x, coverage_y, 'ro');
  xlabel("pixels");
  ylabel("pixels");
  xlim([0, 640]);
  ylim([0, 480]);
  set(gca(), "ydir", "reverse");
  set(gca(), "xaxislocation", "top");

  ginput();
endfunction

function inspect_detection(image_paths, grid_data)
  figure();
  hold on;
  for i = 1:length(image_paths)
    keypoints = grid_data{i}.keypoints;
    img = imread(image_paths{i});
    imshow(img);
    plot(keypoints(1, :), keypoints(2, :), "r+", "linewidth", 1.0);
    title(image_paths{i}, 'interpreter', 'none');
    ginput();
  endfor
endfunction

% Load data
T_WF = load_pose(strcat(data_path, "/target0/data.csv"));
[marker_ts, marker_poses] = load_poses(strcat(data_path, "/body0/data.csv"));
camera_ts = load_camera_timestamps(strcat(data_path, "/cam0/data.csv"));
image_paths = load_camera_data(strcat(data_path, "/cam0/data.csv"));
grid_data = load_grids(strcat(data_path, "/grid0"), 0);

plot_pose_capture(marker_ts, marker_poses, camera_ts);
inspect_coverage(grid_data);
inspect_detection(image_paths, grid_data);
plot_scene(T_WF, marker_ts, marker_poses, camera_ts)
