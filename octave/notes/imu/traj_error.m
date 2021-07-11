function traj_error(gnd, est)
  assert(length(gnd.time) == length(est.time));

  err_x = [];
  err_y = [];
  err_z = [];

  for k = 1:length(gnd.time)
    assert(gnd.time(k) == est.time(k));
    gnd_pos = gnd.pos(:, k);
    est_pos = est.pos(:, k);
    err_x = [err_x, gnd_pos(1) - est_pos(1)];
    err_y = [err_y, gnd_pos(2) - est_pos(2)];
    err_z = [err_z, gnd_pos(3) - est_pos(3)];
  endfor

  dist = norm(gnd.pos(:, end) - est.pos(:, end));
  printf("Euclidean distance between start and end: %f\n", dist);

  printf("Error (x, y, z):\n");
  printf("mean: [%f, %f, %f]\n", mean(err_x), mean(err_y), mean(err_z));
  printf("median: [%f, %f, %f]\n", median(err_x), median(err_y), median(err_z));
  printf("var: [%f, %f, %f]\n", var(err_x), var(err_y), var(err_z));
endfunction
