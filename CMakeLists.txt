CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(prototype)

# C++ COMPILER SETTINGS
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_BUILD_TYPE RELEASE)
# SET(CMAKE_BUILD_TYPE DEBUG)

# DEPENDENCIES
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE(cmake/ImportEigen3.cmake)

# INCLUDES
INCLUDE_DIRECTORIES(include /usr/include/eigen3 ${SUITESPARSE_INCLUDE_DIRS})

# LIBRARY
ADD_LIBRARY(
  ${PROJECT_NAME}
  # control
  src/control/att_ctrl.cpp
  src/control/carrot_ctrl.cpp
  src/control/mission.cpp
  src/control/pid.cpp
  src/control/pos_ctrl.cpp
  src/control/wp_ctrl.cpp
  # core
  src/core/quaternion/jpl.cpp
  src/core/config.cpp
  src/core/data.cpp
  src/core/euler.cpp
  src/core/file.cpp
  src/core/gps.cpp
  src/core/linalg.cpp
  src/core/math.cpp
  src/core/vision.cpp
  src/core/stats.cpp
  src/core/time.cpp
  # dataset
  src/dataset/euroc/camera_data.cpp
  src/dataset/euroc/ground_truth.cpp
  src/dataset/euroc/imu_data.cpp
  src/dataset/euroc/mav_dataset.cpp
  src/dataset/kitti/raw/calib.cpp
  src/dataset/kitti/raw/oxts.cpp
  src/dataset/kitti/raw/parse.cpp
  src/dataset/kitti/raw/raw.cpp
  # driver
  src/driver/camera/camera.cpp
  src/driver/gimbal/sbgc.cpp
  src/driver/imu/mpu6050.cpp
  # src/driver/imu/mpu9250.cpp
  src/driver/pwm/pca9685.cpp
  src/driver/i2c.cpp
  src/driver/uart.cpp
  # model
  src/model/gimbal.cpp
  src/model/mav.cpp
  src/model/two_wheel.cpp
  # msckf
  # src/msckf/blackbox.cpp
  src/msckf/camera_state.cpp
  # src/msckf/feature_estimator.cpp
  # src/msckf/gmsckf.cpp
  src/msckf/imu_state.cpp
  src/msckf/jacobians.cpp
  # src/msckf/msckf.cpp
  # vision
  src/vision/camera/distortion.cpp
  src/vision/camera/pinhole.cpp
  src/vision/feature2d/draw.cpp
  src/vision/feature2d/grid_fast.cpp
  src/vision/feature2d/grid_good.cpp
  src/vision/util.cpp
)
SET(
  ${PROJECT_NAME}_DEPS
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  yaml-cpp
  pthread
  ceres
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
INSTALL(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
INSTALL(
  DIRECTORY src/
  DESTINATION /usr/local/include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/scripts DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_configs DESTINATION ${PROJECT_BINARY_DIR}/tests)
SET(
  UNITTESTS
  # control-carrot_ctrl_test
  control-mission_test
  control-pid_test
  core-config_test
  core-data_test
  core-file_test
  core-gps_test
  core-linalg_test
  core-math_test
  core-stats_test
  core-time_test
  dataset-euroc-camera_data_test
  dataset-euroc-ground_truth_test
  dataset-euroc-imu_data_test
  dataset-euroc-mav_dataset_test
  dataset-kitti-raw-calib_test
  dataset-kitti-raw-oxts_test
  dataset-kitti-raw-parse_test
  dataset-kitti-raw-raw_test
  driver-camera-camera_test
  model-mav_test
)
FOREACH(TEST ${UNITTESTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)
