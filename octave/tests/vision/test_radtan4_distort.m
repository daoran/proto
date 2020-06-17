addpath(genpath("proto"));
test_passed = true;
nb_tests = 100;

for i = 1:nb_tests
  % Setup radtan coefficients
  k1 = unifrnd(-0.1, 0.1);
  k2 = unifrnd(-0.01, 0.01);
  p1 = unifrnd(-0.001, 0.001);
  p2 = unifrnd(-0.001, 0.001);

  % Form and distort point
  point = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  point0 = radtan4_distort(k1, k2, p1, p2, point);
  J = radtan4_point_jacobian(k1, k2, p1, p2, point);

  % Perform numerical diff to obtain finite difference
  step_size = 1e-6;
  eps = eye(2) * step_size;
  finite_diff = zeros(2, 2);
  for i = 1:2
    point1 = point + eps(1:2, i);
    point1 = radtan4_distort(k1, k2, p1, p2, point1);
    finite_diff(1:2, i) = (point1 - point0) / step_size;
  endfor

  % Compare numerical finite diff and analytical jacobian
  % Make sure the difference between all entries are no large than the
  % numerical diff step size
  if any(all((finite_diff - J) > step_size)) == 1
    printf("Invalid Jacobian?\n")
    finite_diff;
    J;
    test_passed = false;
  endif
endfor

if test_passed == false
  exit(-1);
endif
