addpath(genpath("prototype"));

function [ii_image] = illum_invar(image, alpha)
  red = image(:, :, 1);
  green = image(:, :, 2);
  blue = image(:, :, 3);
  ii_image = 0.5 + log(green) - alpha * log(blue) - (1.0 - alpha) * log(red);
endfunction

data_path = "/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/image_02/data";
images = list_dir(data_path);

nb_images = length(images);
for i = 1:nb_images
  image_path = join_paths(data_path, images(i).name);
  image = imread(image_path);
  alpha = 0.9;
  ii_image = illum_invar(image, alpha);
  imshow(ii_image);
  ginput();
endfor

% image = imread("./tests/test_data/lena.jpg");
% alpha = 0.3;
% ii_image = illum_invar(image, alpha);
% imshow(ii_image);
% ginput();
