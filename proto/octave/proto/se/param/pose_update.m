function pose = pose_update(pose, dx)
  assert(isstruct(pose))
  assert(strcmp(pose.type, "pose") || strcmp(pose.type, "extrinsics"));
  assert(length(dx) == 6);

  % Convert pose vector to 4x4 transformation matrix
  % then extract translation and rotation
  T = tf(pose.param);
  r = tf_trans(T);
  q = tf_quat(T);

  % Update translation
  dr = dx(1:3);
  r_kp1 = r + dr;

  % Update rotation
  dalpha = dx(4:6);
  dq = quat_delta(dalpha);
  % q_kp1 = quat_mul(dq, q);  % Pre-multiply perturbation
  q_kp1 = quat_mul(q, dq);  % Post-multiply perturbation

  % Update pose
  T_kp1 = tf(q_kp1, r_kp1);
  pose.param = tf_param(T_kp1);
endfunction
