addpath(genpath("prototype"));

% Setup camera data
data_path = "/data/euroc_mav/MH_01_easy";
data = load_euroc(data_path);

T_SC0 = data.cam0.T_BS;
T_SC1 = data.cam1.T_BS;
T_C0C1 = inv(T_SC0) * T_SC1;

tracker = feature_tracker_create();
tracker.T_C0C1 = T_C0C1;

cam0_image = cv.imread(data.cam0.image_paths{1});
cam1_image = cv.imread(data.cam1.image_paths{1});
feature_tracker_update(tracker, cam0_image, cam1_image, true);

% % Iterate through images
% show_images = true;
% for i = 1:10
%   cam0_image = cv.imread(data.cam0.image_paths{i});
%   cam1_image = cv.imread(data.cam1.image_paths{i});
%   feature_tracker_update(tracker, cam0_image, cam1_image, show_images);
% endfor
