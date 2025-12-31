cmake_minimum_required(VERSION 3.0)
project(ethz_apriltag)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-fPIC -O3)
include_directories(include)

add_library(${PROJECT_NAME}
  src/Edge.cc
  src/FloatImage.cc
  src/GLine2D.cc
  src/GLineSegment2D.cc
  src/Gaussian.cc
  src/GrayModel.cc
  src/Homography33.cc
  src/MathUtil.cc
  src/Quad.cc
  src/Segment.cc
  src/TagDetection.cc
  src/TagDetector.cc
  src/TagFamily.cc
  src/UnionFindSimple.cc
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen)
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)
