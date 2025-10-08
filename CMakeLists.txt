cmake_minimum_required(VERSION 3.0.0)
project(xyz)

# Compilation Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(USE_ADDRESS_SANITIZER OFF)
add_compile_options(-Wall -Wextra)

# Address Sanitizer
if (USE_ADDRESS_SANITIZER)
  message(AUTHOR_WARNING "Building with ASAN - Executables will be slower!")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  link_libraries(-fsanitize=address -static-libasan) # Use -static-libsan for Clang
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)

# Build libyac
link_directories(${CMAKE_PREFIX_PATH}/lib)
include_directories(lib)
add_library(
  ${PROJECT_NAME}
  STATIC
  lib/calib/AprilGrid.cpp
  lib/calib/CalibCamera.cpp
  lib/calib/CalibCameraImu.cpp
  lib/calib/CalibData.cpp
  lib/calib/CalibTarget.cpp
  lib/calib/CalibView.cpp
  lib/calib/CameraChain.cpp
  lib/calib/SolvePnp.cpp
  lib/camera/CameraGeometry.cpp
  lib/camera/CameraModel.cpp
  lib/ceres/ImuError.cpp
  lib/ceres/ParamBlock.cpp
  lib/ceres/PoseManifold.cpp
  lib/ceres/ReprojectionError.cpp
  lib/ceres/ResidualBlock.cpp
  lib/imu/ImuBuffer.cpp
  lib/imu/ImuGeometry.cpp
  lib/imu/ImuState.cpp
  lib/sim/Sim.cpp
  lib/timeline/CalibTargetEvent.cpp
  lib/timeline/CameraEvent.cpp
  lib/timeline/ImuEvent.cpp
  lib/timeline/Timeline.cpp
  lib/timeline/TimelineEvent.cpp
  lib/Core.cpp
)
set(DEPS
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  ${OpenCV_LIBS}
  apriltag::apriltag
  Ceres::ceres
)
target_link_libraries(${PROJECT_NAME} ${DEPS})

# Build unit-tests
set(TEST_DEPS GTest::gtest_main ${PROJECT_NAME} ${DEPS})
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/lib/tests/test_data)

add_executable(unittests
  lib/tests/calib/TestAprilGrid.cpp
  lib/tests/calib/TestCalibCamera.cpp
  # lib/tests/calib/TestCalibCameraImu.cpp
  lib/tests/calib/TestCalibData.cpp
  lib/tests/calib/TestCameraChain.cpp
  lib/tests/camera/TestCameraModel.cpp
  lib/tests/ceres/TestImuError.cpp
  lib/tests/ceres/TestReprojectionError.cpp
  lib/tests/sim/TestSim.cpp
  lib/tests/timeline/TestTimeline.cpp
)
add_compile_definitions(unittests PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(unittests ${TEST_DEPS})
