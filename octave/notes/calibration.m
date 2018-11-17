addpath(genpath("prototype"));

# Settings
step_size = 1e-10;
fdiff_threshold = 1e-4;
nb_tests = 1;

test_failed = false;
for i = 1:nb_tests
  % Rotation and translation of fiducial target w.r.t. world
  rpy_WF = [unifrnd(-0.5, 0.5); unifrnd(-0.5, 0.5); unifrnd(-0.5, 0.5)];
  q_WF = euler2quat(rpy_WF);
  R_WF = quat2rot(q_WF);
  t_WF = [unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05)];

  % Rotation and translation of world w.r.t camera 0
  R_C0W = euler321([unifrnd(-0.1, 0.1), unifrnd(-0.1, 0.1), unifrnd(-0.1, 0.1)]);
  t_C0W = [unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05); unifrnd(-0.05, 0.05)];

  % Transform fiducial point --> world frame --> camera frame --> image plane
  p_F = [unifrnd(-0.5, 0.5); unifrnd(-0.5, 0.5); 1.0];
  p_W = (R_WF * p_F) + t_WF;
  p_C = R_C0W * p_W + t_C0W;
  z = [p_C(1) / p_C(3); p_C(2) / p_C(3)];

  % dh__dp_C
  dh__dp_C = zeros(2, 3);
  dh__dp_C(1, 1) = 1.0;
  dh__dp_C(2, 2) = 1.0;
  dh__dp_C(1, 3) = -(p_C(1) / p_C(3));
  dh__dp_C(2, 3) = -(p_C(2) / p_C(3));
  dh__dp_C = 1 / p_C(3) * dh__dp_C;

  % dp_C__dp_W
  dp_C__dp_W = R_C0W;

  % dh__dp_W
  dh__dp_W = dh__dp_C * dp_C__dp_W;

  % dh__dr_WF
  dh__dr_WF = dh__dp_W * eye(3);

  % dh__dq_WF
  dh__dq_WF = dh__dp_W * -1 * skew(R_WF * p_F);

  % Check: dp_C__dp_W
  % Perform numerical diff to obtain finite difference
  step = eye(3) * step_size;
  finite_diff = zeros(3, 3);
  for i = 1:3
    p_W_diff = p_W + step(1:3, i);
    p_C_diff = (R_C0W * p_W_diff) + t_C0W;
    finite_diff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  J = dp_C__dp_W;
  if any(all((finite_diff - J) > fdiff_threshold)) == 1
    printf("dp_C__dp_W is bad!");
    test_failed = true;
    dp_C__dp_W
    finite_diff
    break;
  endif



  % Check: dh__dp_W
  % Perform numerical diff to obtain finite difference
  step = eye(3) * step_size;
  finite_diff = zeros(2, 3);
  for i = 1:3
    p_W_diff = p_W + step(1:3, i);
    p_C_diff = (R_C0W * p_W_diff) + t_C0W;

    z_hat = [p_C_diff(1) / p_C_diff(3); p_C_diff(2) / p_C_diff(3)];
    finite_diff(1:2, i) = (z_hat - z) / step_size;
  endfor

  J = dh__dp_W;
  if any(all((finite_diff - J) > fdiff_threshold)) == 1
    printf("dh__dp_W is bad!");
    test_failed = true;
    dh__dp_W
    finite_diff
    break;
  endif


  % Check: dh__dr_WF
  % Perform numerical diff to obtain finite difference
  step = eye(3) * step_size;
  finite_diff = zeros(2, 3);
  for i = 1:3
    t_WF_diff = t_WF + step(1:3, i);
    p_W_diff = (R_WF * p_F) + t_WF_diff;
    p_C_diff = (R_C0W * p_W_diff) + t_C0W;

    z_hat = [p_C_diff(1) / p_C_diff(3); p_C_diff(2) / p_C_diff(3)];
    finite_diff(1:2, i) = (z_hat - z) / step_size;
  endfor

  J = dh__dr_WF;
  if any(all((finite_diff - J) > fdiff_threshold)) == 1
    printf("dh__dr_WF is bad!");
    test_failed = true;
    dh__dr_WF
    finite_diff
    break;
  endif



  % Check: dh__dR_WF
  % Perform numerical diff to obtain finite difference
  rvec = eye(3) * step_size;
  finite_diff = zeros(2, 3);
  for i = 1:3
    R_WF_diff = rvec2rot(rvec(1:3, i));
    p_W_diff = (R_WF_diff * R_WF * p_F) + t_WF;
    p_C_diff = (R_C0W * p_W_diff) + t_C0W;

    z_hat = [p_C_diff(1) / p_C_diff(3); p_C_diff(2) / p_C_diff(3)];
    finite_diff(1:2, i) = (z_hat - z) / step_size;
  endfor

  J = dh__dq_WF;
  if any(all((finite_diff - J) > fdiff_threshold)) == 1
    printf("dh__dq_WF is bad!\n");
    test_failed = true;
    dh__dq_WF
    finite_diff
    break;
  endif

endfor

if test_failed
  printf("Test failed!");
else
  printf("All good!");
endif
