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
  lib/ImuError.cpp
  lib/ParamBlock.cpp
  lib/PoseManifold.cpp
  lib/ReprojectionError.cpp
  lib/ResidualBlock.cpp
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
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/tests/test_data)

add_executable(TestAprilGrid lib/tests/TestAprilGrid.cpp)
add_compile_definitions(TestAprilGrid PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestAprilGrid ${TEST_DEPS})

add_executable(TestCalibCamera lib/tests/TestCalibCamera.cpp)
add_compile_definitions(TestCalibCamera PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCalibCamera ${TEST_DEPS})

# add_executable(TestCalibCameraImu lib/tests/TestCalibCameraImu.cpp)
# add_compile_definitions(TestCalibCameraImu PRIVATE TEST_DATA="${TEST_DATA}")
# target_link_libraries(TestCalibCameraImu ${TEST_DEPS})

add_executable(TestCalibData lib/tests/TestCalibData.cpp)
add_compile_definitions(TestCalibData PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCalibData ${TEST_DEPS})

add_executable(TestCameraChain lib/tests/TestCameraChain.cpp)
add_compile_definitions(TestCameraChain PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraChain ${TEST_DEPS})

add_executable(TestCameraModel lib/tests/TestCameraModel.cpp)
add_compile_definitions(TestCameraModel PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraModel ${TEST_DEPS})

add_executable(TestImuError lib/tests/TestImuError.cpp)
add_compile_definitions(TestImuError PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestImuError ${TEST_DEPS})

add_executable(TestReprojectionError lib/tests/TestReprojectionError.cpp)
add_compile_definitions(TestReprojectionError PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestReprojectionError ${TEST_DEPS})

add_executable(TestSim lib/tests/TestSim.cpp)
add_compile_definitions(TestSim PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestSim ${TEST_DEPS})

add_executable(TestTimeline lib/tests/TestTimeline.cpp)
add_compile_definitions(TestTimeline PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestTimeline ${TEST_DEPS})
