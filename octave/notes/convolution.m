pkg load image;

% I = imread("tests/test_data/chessboard.png");
% I = rgb2gray(I);
% % I = [1:4; 5:8; 9:12; 13:16];
%
% kern_size = 3;
% extending_edge = true;
% % Horizontal line kernel
% kern = [-1, -1, -1;
% 			  2,  2,  2;
% 				-1, -1, -1];
% % Vertical line kernel
% % kern = [-1, 2, -1;
% % 			  -1, 2, -2;
% % 				-1, 2, -1];
% assert(rows(kern) == kern_size);
% padding = floor(kern_size / 2);
%
% % Calculate working image size with padding
% [r, c, channels] = size(I);
% r = r + (padding * 2);
% c = c + (padding * 2);
%
% % Obtain corner values
% val_tl = I(1, 1);
% val_tr = I(1, end);
% val_br = I(end, end);
% val_bl = I(end, 1);
%
% % Obtain top, bottom, left and right blocks
% I_top = repmat(I(1, 1:end), padding, 1);
% I_btm = repmat(I(end, 1:end), padding, 1);
% I_lft = repmat(I(:, 1), 1, padding);
% I_rht = repmat(I(:, end), 1, padding);
%
% % Obtain corner blocks
% I_top_left = repmat(val_tl, padding, padding);
% I_btm_left = repmat(val_bl, padding, padding);
% I_top_right = repmat(val_tr, padding, padding);
% I_btm_right = repmat(val_br, padding, padding);
%
% % Form padding dimensions
% ptop_dim = [padding, c];
% pbtm_dim = [padding, c];
% plft_dim = [r, padding];
% prht_dim = [r, padding];
%
% % Form working image
% W = zeros(r, c);
% % -- Copy image to center of working image
% W(padding+1:r-padding, padding+1:c-padding) = I;
% % -- Edge handing (extending)
% if extending_edge
% 	% ---- Copy top, bottom, left and right block
% 	W(1:padding, 1+padding:end-padding) = I_top;
% 	W(end-padding+1:end, 1+padding:end-padding) = I_btm;
% 	W(1+padding:end-padding, 1:padding) = I_lft;
% 	W(1+padding:end-padding, end-padding+1:end) = I_rht;
% 	% ---- Copy corner blocks
% 	W(1:padding, 1:padding) = I_top_left;
% 	W(end-padding+1:end, 1:padding) = I_btm_left;
% 	W(1:padding, end-padding+1:end) = I_top_right;
% 	W(end-padding+1:end, end-padding+1:end) = I_btm_right;
% endif
%
% % Convolve
% output = zeros(size(I));
% for i = 1:rows(W)-(padding*2)
% 	if mod(i, 30) == 0
% 		printf("progress: %.2f%%\n", (i / rows(W)) * 100.0);
% 	endif
%
% 	for j = 1:columns(W)-(padding*2)
% 		% Multiply kernel with working image W
% 		W_sub = W(i:i+kern_size-1, j:j+kern_size-1);
% 		pixel = sum(sum(W_sub .* kern));
% 		output(i, j) = pixel;
% 	endfor
% endfor
% printf("progress: 100%%\n");
%
% imshow(output);
% ginput();


im = imread("tests/test_data/chessboard.png");
im = rgb2gray(im);

line_horiz_kern = [-1, -1, -1;
						 			  2,  2,  2;
						 			 -1, -1, -1];
% line_vert_kern = [-1, 2, -1;
% 						 			-1, 2, -1;
% 						 			-1, 2, -1];
% line_45_kern = [-1, -1,  2;
% 								-1,  2, -1;
% 							   2, -1, -1];
% line_135_kern = [ 2, -1, -1;
% 								 -1,  2, -1;
% 							   -1, -1,  2];
% edge_kern = [-1, -1, -1;
% 						 -1,  8, -1;
% 						 -1, -1, -1];
% sharpen_kern = [0, -1, 0;
% 			  			  -1, 5, -1;
% 							  0, -1, 0];
% gauss_kern = 1 / 16 * [0, 2, 0;
% 											 2, 4, 2;
% 											 0, 2, 0];
% blur_kern = 1 / 9 * [1, 1, 1;
% 										 1, 1, 1;
% 										 1, 1, 1];

imshow(imfilter(im, line_horiz_kern));
% imshow(imfilter(im, line_vert_kern));
% imshow(imfilter(im, line_45_kern));
% imshow(imfilter(im, line_135_kern));
% imshow(imfilter(im, edge_kern));
% imshow(imfilter(im, sharpen_kern));
% imshow(imfilter(im, gauss_kern));
% imshow(imfilter(im, blur_kern));
ginput();
