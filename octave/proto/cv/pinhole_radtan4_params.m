function params = pinhole_radtan4_params(camera)
  params = [camera.proj_params; camera.dist_params];
endfunction
