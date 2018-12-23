#!/bin/bash
set -e  # Exit on first error

run_test() {
  echo -n "TEST [$1] "

  if octave "$1"; then
    echo 'PASSED!'
  else
    echo 'FAILED!'
  fi
}

# os
run_test tests/os/test_join_paths.m
run_test tests/os/test_list_dir.m

# plot
run_test tests/plot/test_draw_camera.m
run_test tests/plot/test_draw_frame.m
run_test tests/plot/test_draw_points.m

# util
run_test tests/util/test_config.m
run_test tests/util/test_bezier_cubic.m

# vision
run_test tests/vision/test_camera_measurements.m
run_test tests/vision/test_chessboard_create.m
run_test tests/vision/test_radtan4_distort.m
run_test tests/vision/test_radtan4_undistort.m
