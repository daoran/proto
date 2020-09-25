function pose = pose_update(pose, dx)
  assert(isstruct(pose) && pose.type == "pose");
  assert(length(dx) == 6);

  dalpha = dx(1:3);
  dq = quat_delta(dalpha);
  pose.param(1:4) = quat_mul(dq, pose.param(1:4));

  dr = dx(4:6);
  pose.param(5:7) = pose.param(5:7)+ dr;
endfunction
