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
  point1 = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  point_distorted = radtan4_distort(k1, k2, p1, p2, point1);
  point2 = radtan4_undistort(k1, k2, p1, p2, point_distorted);

  if norm(point1 - point2) > 1e-10
    test_passed = false;
  endif
endfor

if test_passed == false
  exit(-1);
endif
