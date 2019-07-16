CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(prototype)

# C++ COMPILER SETTINGS
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(BUILD_SHARED_LIBS ON)

# DEPENDENCIES
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)
FIND_PACKAGE(GeographicLib 1.34 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE(cmake/ImportEigen3.cmake)
if (OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SUITESPARSE_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# # IMGUI
# INCLUDE_DIRECTORIES(./deps/imgui)
# ADD_LIBRARY(
#   imgui
#   deps/imgui/imgui.cpp
#   deps/imgui/imgui_draw.cpp
#   deps/imgui/imgui_widgets.cpp
# )

# LIBRARY
ADD_LIBRARY(
  ${PROJECT_NAME}
  # calib
  src/calib/aprilgrid.cpp
  src/calib/calib_data.cpp
  src/calib/calib_camera.cpp
  src/calib/calib_camera_nbv.cpp
  # src/calib/calib_gimbal.cpp
  src/calib/calib_stereo.cpp
  src/calib/calib_vicon_marker.cpp
  src/calib/chessboard.cpp
  # comm
  src/comm/tcp.cpp
  # control
  src/control/carrot_ctrl.cpp
  src/control/pid.cpp
  # core
  src/core/config.cpp
  src/core/data.cpp
  src/core/file.cpp
  src/core/gps.cpp
  src/core/math.cpp
  src/core/tf.cpp
  src/core/time.cpp
  # dataset
  src/dataset/euroc.cpp
  src/dataset/kitti.cpp
  src/dataset/timeline.cpp
  # driver
  src/driver/camera.cpp
  src/driver/i2c.cpp
  src/driver/mpu6050.cpp
  src/driver/pca9685.cpp
  src/driver/sbgc.cpp
  src/driver/ublox.cpp
  src/driver/uart.cpp
  # estimation
  src/estimation/filter.cpp
  src/estimation/measurement.cpp
  # mav
  src/mav/atl.cpp
  src/mav/att_ctrl.cpp
  src/mav/lz.cpp
  src/mav/mission.cpp
  src/mav/pos_ctrl.cpp
  src/mav/tk_ctrl.cpp
  # src/mav/wp_ctrl.cpp
  # model
  src/model/gimbal.cpp
  src/model/mav.cpp
  src/model/two_wheel.cpp
  # vision
  src/vision/camera/radtan.cpp
  src/vision/camera/equi.cpp
  src/vision/camera/pinhole.cpp
  src/vision/feature2d/draw.cpp
  src/vision/feature2d/grid_fast.cpp
  src/vision/feature2d/grid_good.cpp
  src/vision/frontend.cpp
  src/vision/vision_common.cpp
)
SET(
  ${PROJECT_NAME}_DEPS
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
  apriltags
  ${GeographicLib_LIBRARIES}
  # imgui
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
# TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)
INSTALL(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
INSTALL(
  DIRECTORY include/
  DESTINATION /usr/local/include
  FILES_MATCHING PATTERN "*.hpp"
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/scripts DESTINATION ${PROJECT_BINARY_DIR}/tests)
SET(
  UNITTESTS
  # calib-test_aprilgrid
  # calib-test_calib_camera
  # calib-test_calib_camera_nbv
  # calib-test_calib_data
  # # calib-test_calib_gimbal
  # calib-test_calib_stereo
  # calib-test_chessboard
  # comm-test_tcp
  # control-test_carrot_ctrl
  # control-test_pid
  # core-test_config
  # core-test_data
  # core-test_file
  # core-test_gps
  # core-test_math
  # core-test_tf
  # core-test_time
  # dataset-test_euroc
  # dataset-test_kitti
  # driver-test_camera
  # driver-test_ublox
  # driver-ublox_base_station_tester
  estimation-test_measurement
  # # model-test_mav
  # model-test_two_wheel
  # mav-test_atl
  # # mav-test_att_ctrl
  # mav-test_lz
  # mav-test_mission
  # # mav-test_pos_ctrl
  # mav-test_wp_ctrl
  # vision-camera-test_camera_geometry
  # vision-camera-test_equi
  # vision-camera-test_pinhole
  # vision-camera-test_radtan
  # vision-feature2d-test_grid_fast
  # vision-feature2d-test_grid_good
  vision-test_frontend
  # vision-test_vision_common
)
FOREACH(TEST ${UNITTESTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)

# # APPS
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/apps)
# FILE(COPY configs DESTINATION ${PROJECT_BINARY_DIR}/apps)
# SET(
#   APP_DEPS
#   prototype
#   ${OpenCV_LIBS}
#   ${SUITESPARSE_LIBRARIES}
#   yaml-cpp
#   apriltags
#   ceres
#   ${GeographicLib_LIBRARIES}
# )
# SET(
#   APPS
#   calib_camera
#   calib_stereo
#   # calib_vicon_marker
#   detect_aprilgrid
#   ublox_base_station
#   ublox_rover
#   validate_intrinsics
#   validate_stereo
# )
# FOREACH(APP ${APPS})
#   ADD_EXECUTABLE(${APP} src/apps/${APP}.cpp)
#   TARGET_LINK_LIBRARIES(${APP} ${APP_DEPS})
# ENDFOREACH(APP)
