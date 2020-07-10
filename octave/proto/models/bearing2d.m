function b = bearing2d(mx, my, x, y, theta)
  b = mod(atan2(my - y, mx - x) - theta + pi, 2 * pi) - pi;
end
