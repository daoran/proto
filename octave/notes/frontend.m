addpath(genpath("."));

% Load KITTI Raw data sequence
data_path = "/data/kitti";
data_date = "2011_09_26";
data_seq = "0005";
data = load_kitti(data_path, data_date, data_seq);



inited = false;
kps = [];
min_kps = 100;
prev_img = [];


for i = 1:length(data.cam0)
  img = imread(data.cam0{i});

  % Add new kps to track
  if length(kps) < min_kps
    kps = [kps; cv.goodFeaturesToTrack(img)];
  endif

  % Mark frontend as initialized
  if inited == false
    inited = true;
    prev_img = img;
    continue;
  endif

  % Track features
  [kps_tracked, status, err] = cv.calcOpticalFlowPyrLK(prev_img, img, kps);
  kps = [];
  for j = 1:length(kps_tracked)
    if status(j)
      kps = [kps; kps_tracked(j)];
    endif
  endfor

  % Update
  prev_img = img;
  printf("frame: %d\n", i);
  printf("nb_kps: %d\n", length(kps));
  % pause();

  img = cv.cvtColor(img, "GRAY2BGR");
  for j = 1:length(kps_tracked)
    img = cv.circle(img, kps_tracked{j}, 2, "Color", [255, 0, 0]);
  endfor
  imshow(img);
  ginput();
endfor
