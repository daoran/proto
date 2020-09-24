function data = calib_data_add_noise(data)
  nb_poses = length(data.time);

  % Add noise to camera position and rotation
  for i = 1:nb_poses
    % Position
    % data.r_WC{i} += normrnd([0; 0; 0], 1e-2);
    data.r_WC{i} += normrnd([0; 0; 0], 1e-1);

    % Rotation
    dq = quat_delta(normrnd([0; 0; 0], 1e-1));
    q_WC = data.q_WC{i};
    data.q_WC{i} = quat_mul(dq, q_WC);
  endfor

  % Add noise to point data
  data.p_data += normrnd(zeros(3, length(data.p_data)), 1e-2);
endfunction

