function [ii_image] = illum_invar(image, alpha)
  red = image(:, :, 1);
  green = image(:, :, 2);
  blue = image(:, :, 3);
  ii_image = 0.5 + log(green) - alpha * log(blue) - (1.0 - alpha) * log(red);
endfunction
