addpath(genpath("prototype"));
pkg load geometry;

% Setup camera data
data_path = "/data/euroc_mav/MH_01_easy";
data = load_euroc(data_path);

cam0_image = cv.imread(data.cam0.image_paths{1});
cam0_kps = cv.FAST(cam0_image, "Threshold", 100.0);

cam0_kps(2)
cam0_kps(3)

% patch_size = 7;
% p = cam0_kps(2).pt
% top_left = [p(1) - (patch_size / 2.0); p(2) - (patch_size / 2.0)]
% top_right = [p(1) + (patch_size / 2.0); p(2) - (patch_size / 2.0)]
% bottom_left = [p(1) - (patch_size / 2.0); p(2) + (patch_size / 2.0)]
% bottom_right = [p(1) + (patch_size / 2.0); p(2) + (patch_size / 2.0)]
%
%
% % cam0_image = cv.drawKeypoints(cam0_image, cam0_kps, "Color", [255, 0, 0]);
% % size(cam0_image)
% cam0_image(p(2), p(1), 1) = 255.0;
% imshow(cam0_image);
% hold on;
% rect_pos = [top_left(1), top_left(2), patch_size, patch_size];
% rectangle('Position', rect_pos,...
% 					'EdgeColor', 'r',...
% 					'LineWidth', 3,...
% 					'LineStyle','-');
% ginput();
