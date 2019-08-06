function [r_ellipse, X0, Y0] = error_ellipse_2d(x, y, chisq_val=0)
  % -- Calculate the eigenvectors and eigenvalues
  data = [x y];
  covariance = cov(data);
  [eigenvec, eigenval] = eig(covariance);

  % -- Get the index of the largest eigenvector
  [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
  largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

  % -- Get the largest eigenvalue
  largest_eigenval = max(max(eigenval));

  % -- Get the smallest eigenvector and eigenvalue
  if (largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2))
    smallest_eigenvec = eigenvec(:,2);
  else
    smallest_eigenval = max(eigenval(:,1))
    smallest_eigenvec = eigenvec(1,:);
  end

  % Calculate the angle between the x-axis and the largest eigenvector
  angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
  % -- This angle is between -pi and pi.
  % -- Let's shift it such that the angle is between 0 and 2pi
  if (angle < 0)
    angle = angle + 2*pi;
  end
  % -- Get the coordinates of the data mean
  avg = mean(data);
  X0 = avg(1);
  Y0 = avg(2);
  % -- Get the 95% confidence interval error ellipse
  if chisq_val == 0
    chisq_val = sqrt(5.99);  % 95% confidence level
    % chisq_val = sqrt(4.61);  % 90% confidence level
  endif
  theta_grid = linspace(0, 2*pi);
  phi = angle;
  a = chisq_val * sqrt(largest_eigenval);
  b = chisq_val * sqrt(smallest_eigenval);
  % -- Ellipse in x and y coordinates
  ellipse_x_r  = a*cos( theta_grid );
  ellipse_y_r  = b*sin( theta_grid );
  % -- Define a rotation matrix
  R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
  % -- Let's rotate the ellipse to some angle phi
  r_ellipse = [ellipse_x_r; ellipse_y_r]' * R;
endfunction
