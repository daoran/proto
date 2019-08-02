addpath(genpath("proto"));
pkg load image;
graphics_toolkit("fltk");

% Settings
k = 0.04;
thresh = 10;
radius = 5;
N = 100;
subpixel = true;

% Load image
im = imread("tests/test_data/chessboard.png");
im = rgb2gray(im);

% Calculate image gradients
[Ix, Iy] = derivative5(im, 'x', 'y');
gauss_kern = 1 / 16 * [0, 2, 0;
											 2, 4, 2;
											 0, 2, 0];
Ix2 = imfilter(Ix.^2, gauss_kern);
Iy2 = imfilter(Iy.^2, gauss_kern);
Ixy = imfilter(Ix.*Iy, gauss_kern);

% Calculate Harris corner response
R = (Ix2.*Iy2 - Ixy.^2) - k*(Ix2 + Iy2).^2;

% Non-max Suppression
[rows, cols] = size(R);

% Extract local maxima by performing a grey scale morphological
% dilation and then finding points in the corner strength image that
% match the dilated image and are also greater than the threshold.
sze = 2.0 * radius + 1.0;  % Size of dilation mask.
% mx = imdilate(R, 1.0);
mx = ordfilt2(R, sze^2,ones(sze)); % Grey-scale dilate.

% Make mask to exclude points within radius of the image boundary.
bordermask = zeros(size(R));
bordermask(radius+1:end-radius, radius+1:end-radius) = 1;

% Find maxima, threshold, and apply bordermask
Rmx = (R==mx) & (R>thresh) & bordermask;
[r, c] = find(Rmx);  % Find row,col coords.
keypoints = [c'; r'];

% Visualize
figure();
subplot(411)
imshow(im);
title("Image");

subplot(412)
imshow(Ix);
title("Ix");

subplot(413)
imshow(Iy);
title("Iy");

subplot(414)
imshow(R);
title("R");

figure();
hold on;
imshow(im);
plot(keypoints(1, :), keypoints(2, :), "r+", "linewidth", 2.0);
title("Detected Corners");

ginput();
