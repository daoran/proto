CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(proto)

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
FIND_PACKAGE(opengv REQUIRED)
FIND_PACKAGE(GeographicLib 1.34 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE(cmake/ImportEigen3.cmake)

SET(OpenGL_GL_PREFERENCE GLVND)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(glfw3 REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
FIND_PACKAGE(glm REQUIRED)

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

# GLAD
INCLUDE_DIRECTORIES(/usr/local/src/glad/include)

# IMGUI
INCLUDE_DIRECTORIES(/usr/local/src/imgui /usr/local/src/imgui/examples)
ADD_DEFINITIONS(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)
ADD_LIBRARY(
  imgui
  /usr/local/src/imgui/imgui.cpp
  /usr/local/src/imgui/imgui_draw.cpp
  /usr/local/src/imgui/imgui_demo.cpp
  /usr/local/src/imgui/imgui_widgets.cpp
)
ADD_LIBRARY(
  imgui_impl
  /usr/local/src/imgui/examples/imgui_impl_glfw.cpp
  /usr/local/src/imgui/examples/imgui_impl_opengl3.cpp
)

# # BUILD WITH CATKIN
# IF(CATKIN_DEVEL_PREFIX)
#   FIND_PACKAGE(catkin REQUIRED)
#   CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES proto)
# ENDIF()

SET(
  ${PROJECT_NAME}_DEPS
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
  apriltags
  ${GeographicLib_LIBRARIES}
  glad
  glfw
  dl
  ${ASSIMP_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
	opengv
  imgui
  imgui_impl
)

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
  # core
  src/core/core.cpp
  src/core/stb.cpp
  # dataset
  src/dataset/euroc.cpp
  src/dataset/kitti.cpp
  src/dataset/timeline.cpp
  # estimation
  src/estimation/factor.cpp
  src/estimation/filter.cpp
  src/estimation/frontend.cpp
  # mav
  src/mav/atl.cpp
  # model
  src/model/gimbal.cpp
  src/model/mav.cpp
  src/model/two_wheel.cpp
  # vision
  src/vision/camera/camera.cpp
  src/vision/feature2d/draw.cpp
  src/vision/feature2d/grid_fast.cpp
  src/vision/feature2d/grid_good.cpp
  src/vision/vision_common.cpp
  # viz
  src/viz/draw.cpp
  src/viz/camera.cpp
  src/viz/gui.cpp
  src/viz/model.cpp
  src/viz/mesh.cpp
  src/viz/shader.cpp
  src/viz/texture.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
# TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)
INSTALL(
  TARGETS ${PROJECT_NAME} imgui imgui_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
INSTALL(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h*"
)
INSTALL(
  FILES cmake/ProtoConfig.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/Proto
)
INSTALL(
  FILES cmake/SuiteSparseConfig.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/SuiteSparse
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/scripts DESTINATION ${PROJECT_BINARY_DIR}/tests)
SET(
  UNITTESTS
  calib-test_aprilgrid
  calib-test_calib_camera
  calib-test_calib_camera_nbv
  calib-test_calib_data
  # calib-test_calib_gimbal
  calib-test_calib_stereo
  calib-test_sandbox
  calib-test_sandbox2
  core-test_core
  dataset-test_euroc
  dataset-test_kitti
  estimation-test_factor
  # model-test_mav
  model-test_two_wheel
  mav-test_atl
  vision-camera-test_camera
  vision-feature2d-test_grid_fast
  vision-feature2d-test_grid_good
  # vision-test_frontend
  vision-test_vision_common
  viz-test_gui
  viz-test_plane
)
FOREACH(TEST ${UNITTESTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)
