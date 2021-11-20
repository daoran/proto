function [p] = sphere(rho, theta, phi)
  % rho: Sphere radius
  % theta: longitude [rad]
  % phi: Latitude [rad]

  x = rho * sin(theta) * cos(phi) ;
  y = rho * sin(theta) * sin(phi) ;
  z = rho * cos(theta);
  p = [x; y; z];
endfunction
