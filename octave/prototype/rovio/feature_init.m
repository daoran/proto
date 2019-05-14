function feature = feature_init()
  feature = {};

	feature.pixel = [];					   % Pixel
	feature.pixel_is_valid = 0;    % Is pixel valid

	feature.bearing = [];  				 % Bearing vector
	feature.bearing_is_valid = 0;  % Is bearing vector valid

	feature.pixel_cov = [];		% Estimated covariance of pixel coordinates.
	feature.eigen_vec1 = [];  % Estimated eigenvector of major uncertainty axis.
	feature.eigen_vec2 = [];  % Estimated eigenvector of minor uncertainty axis.

	% Stdev in the direction of the uncertainty ellipse.
  feature.sigma1 = 0.0;  % Major axis.
  feature.sigma2 = 0.0;  % Semi-major axis.
  feature.sigma_angle = 0.0;  % Angle between x-axis and major axis
endfunction
