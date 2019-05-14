addpath(genpath("prototype"));
% pkg load symbolic;
%
% syms P11 P12 P13 P14;
% syms P21 P22 P23 P24;
% syms P31 P32 P33 P34;
% syms Xx Xy Xz Xw;
% syms x y z;
%
% P = [P11, P12, P13, P14;
%      P21, P22, P23, P24;
%      P31, P32, P33, P34];
% X = [Xx; Xy; Xz; 1.0];
% z = [x; y; 1.0];

% cross(z, (P * X))

% function linear_triangulation(z, z_dash, P, P_dash)
%   x = z(1);
%   y = z(2);
%   x_dash = z_dash(1);
%   y_dash = z_dash(2);
%
%   P1T = P(1, :)';
%   P2T = P(2, :)';
%   P3T = P(3, :)';
%
%   P1T_dash = P_dash(1, :)';
%   P2T_dash = P_dash(2, :)';
%   P3T_dash = P_dash(3, :)';
%
%   A = [x * P3T - P1T;
%        y * P3T - P2T;
%        x * P3T_dash - P1T_dash;
%        y * P3T_dash - P2T_dash];
%
%   svd(A);
% endfunction

function [ii_image] = illum_invar_transform(image, alpha)
  red = image(:, :, 1);
  green = image(:, :, 2);
  blue = image(:, :, 3);
  ii_image = 0.5 + log(green) - alpha * log(blue) - (1.0 - alpha) * log(red);
endfunction

data_path = "/data/kitti/2011_09_26/2011_09_26_drive_0005_extract/image_02/data";
images = list_dir(data_path);

nb_images = length(images);
for i = 1:nb_images
  image_path = join_paths(data_path, images(i).name);
  image = imread(image_path);
  alpha = 0.1;
  ii_image = illum_invar_transform(image, alpha);
  imshow(ii_image);
  ginput();
endfor

% image = imread("./tests/test_data/lena.jpg");
% alpha = 0.3;
% ii_image = illum_invar_transform(image, alpha);
% imshow(ii_image);
% ginput();
