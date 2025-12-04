cmake_minimum_required(VERSION 3.0.0)
project(xyz)

# Compilation Settings
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(USE_ADDRESS_SANITIZER ON)
add_compile_options(-Wall -Wextra)

# Address Sanitizer
if (USE_ADDRESS_SANITIZER)
  message(AUTHOR_WARNING "Building with ASAN - Executables will be slower!")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  # link_libraries(-fsanitize=address -static-libasan) # g++
  link_libraries(-fsanitize=address -static-libsan) # clang
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_package(rerun_sdk REQUIRED)

# Build libyac
link_directories(${CMAKE_PREFIX_PATH}/lib)
include_directories(lib)
add_library(
  ${PROJECT_NAME}
  STATIC
  lib/calib/AprilGrid.cpp
  lib/calib/AprilGridConfig.cpp
  lib/calib/AprilGridDetector.cpp
  lib/calib/CalibCamera.cpp
  # lib/calib/CalibCameraImu.cpp
  lib/calib/CalibData.cpp
  lib/calib/CalibTarget.cpp
  lib/calib/CalibTargetChain.cpp
  lib/calib/CalibTargetGeometry.cpp
  lib/calib/CameraChain.cpp
  lib/calib/SolvePnp.cpp
  lib/camera/BrownConrady4.cpp
  lib/camera/CameraGeometry.cpp
  lib/camera/KannalaBrandt4.cpp
  lib/camera/Pinhole.cpp
  lib/ceres/ImuError.cpp
  lib/ceres/ParamBlock.cpp
  lib/ceres/PoseManifold.cpp
  lib/ceres/ReprojError.cpp
  lib/ceres/CalibCameraError.cpp
  lib/ceres/CalibCameraImuError.cpp
  lib/ceres/ResidualBlock.cpp
  lib/dataset/EuRoC.cpp
  lib/imu/ImuBuffer.cpp
  lib/imu/ImuGeometry.cpp
  lib/imu/ImuState.cpp
  # lib/sim/SimCalibCamera.cpp
  # lib/sim/SimImu.cpp
  # lib/sim/SimVio.cpp
  lib/timeline/CalibTargetEvent.cpp
  lib/timeline/CameraEvent.cpp
  lib/timeline/ImuEvent.cpp
  lib/timeline/Timeline.cpp
  lib/timeline/TimelineEvent.cpp
  lib/Core.cpp
  lib/Logger.cpp
)
set(DEPS
  yaml-cpp::yaml-cpp
  rerun_sdk
  Eigen3::Eigen
  ${OpenCV_LIBS}
  apriltag::apriltag
  ethz_apriltag
  Ceres::ceres
)
target_link_libraries(${PROJECT_NAME} ${DEPS})

# Build unit-tests
set(TEST_DEPS GTest::gtest_main ${PROJECT_NAME} ${DEPS})
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/lib/test_data)

add_executable(unittests
  lib/calib/TestAprilGrid.cpp
  lib/calib/TestAprilGridDetector.cpp
  lib/calib/TestCalibCamera.cpp
  # lib/calib/TestCalibCameraImu.cpp
  lib/calib/TestCalibData.cpp
  lib/calib/TestCameraChain.cpp
  lib/camera/TestBrownConrady4.cpp
  lib/camera/TestKannalaBrandt4.cpp
  lib/camera/TestPinhole.cpp
  lib/ceres/TestCalibCameraError.cpp
  # lib/ceres/TestCalibCameraImuError.cpp
  # lib/ceres/TestImuError.cpp
  lib/ceres/TestReprojError.cpp
  # lib/sim/TestSimCalibCamera.cpp
  # lib/sim/TestSimImu.cpp
  # lib/sim/TestSimVio.cpp
  # lib/timeline/TestTimeline.cpp
  # lib/TestLogger.cpp
)
add_compile_definitions(unittests PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(unittests ${TEST_DEPS})
