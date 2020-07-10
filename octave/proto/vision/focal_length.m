function fx = focal_length(image_width, fov)
  fx = (image_width / 2.0) / tan(deg2rad(fov / 2.0));
endfunction
