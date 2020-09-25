function [r, jacobians] = imu_factor_eval(factor, params)
  assert(isstruct(factor));
  assert(length(params) == 4);

  % Setup return values
  r = zeros(15, 1);
  jacobians{1} = zeros(15, 6);  % w.r.t sensor pose i
  jacobians{2} = zeros(15, 9);  % w.r.t speed bias i
  jacobians{3} = zeros(15, 8);  % w.r.t sensor pose j
  jacobians{4} = zeros(15, 9);  % w.r.t speed bias j

  % Map params
  pose_i = params{1};
  sb_i = params{2};
  pose_j = params{3};
  sb_j = params{4};

  % -- Sensor pose at timestep i
	T_i = tf(pose_i.param);
	C_i = tf_rot(T_i);
	C_i_inv = C_i';
	q_i = tf_quat(T_i);
	r_i = tf_trans(T_i);
	% -- Speed and bias at timestamp i
	v_i = sb_i.param(1:3);
	ba_i = sb_i.param(4:6);
	bg_i = sb_i.param(7:9);
	% -- Sensor pose at timestep j
	T_j = tf(pose_j.param);
	q_j = tf_quat(T_j);
	r_j = tf_trans(T_j);
	% -- Speed and bias at timestep j
	v_j = sb_j.param(1:3);
	ba_j = sb_j.param(4:6);
	bg_j = sb_j.param(7:9);

	% % Obtain Jacobians for gyro and accel bias
	% dp_dbg = F.block<3, 3>(0, 9);
	% dp_dba = F.block<3, 3>(0, 12);
	% dv_dbg = F.block<3, 3>(3, 9);
	% dv_dba = F.block<3, 3>(3, 12);
	% dq_dbg = F.block<3, 3>(6, 12);

	% Calculate residuals
	% dt_ij = ns2sec(imu_ts.back() - imu_ts.front());
	% dt_ij_sq = dt_ij * dt_ij;
	% dbg = bg_i - bg;
	% dba = ba_i - ba;
	% alpha = dp + dp_dbg * dbg + dp_dba * dba;
	% beta = dv + dv_dbg * dbg + dv_dba * dba;
	% gamma = dq * quat_delta(dq_dbg * dbg);

  % q_i_inv = quat_inv(q_i);
  % q_j_inv = quat_inv(q_j);
  % gamma_inv = quat_inv(gamma);

	% r = [C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha;
	% 		 C_i_inv * (v_j - v_i + g * dt_ij) - beta;
	% 		 2.0 * (gamma_inv * (q_i_inv * q_j)).vec();
	% 		 ba_j - ba_i;
	% 		 bg_j - bg_i];

  % Calculate Jacobians
endfunction
