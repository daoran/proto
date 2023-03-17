CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(proto_ros2)

# Default to C99
IF(NOT CMAKE_C_STANDARD)
	SET(CMAKE_C_STANDARD 99)
ENDIF()

# Default to C++14
IF(NOT CMAKE_CXX_STANDARD)
	SET(CMAKE_CXX_STANDARD 14)
ENDIF()

IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	ADD_COMPILE_OPTIONS(-Wall -Wextra -Wpedantic)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(builtin_interfaces REQUIRED)
FIND_PACKAGE(eigen3_cmake_module REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)
FIND_PACKAGE(px4_msgs REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(sensor_msgs REQUIRED)

###########
# TARGETS #
###########

# examples/offboard/offboard_control
# add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
# ament_target_dependencies(offboard_control rclcpp px4_msgs)
# install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

###########
# INSTALL #
###########

# EXPORT DEPENDENCIES
ament_export_dependencies(
	ament_cmake
	rclcpp
	rosidl_default_runtime
	eigen3_cmake_module
	Eigen3
	px4_msgs
	geometry_msgs
	sensor_msgs
)
# ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)
# ament_export_libraries(frame_transforms)

# INSTALL HEADERS
# INSTALL(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
# INSTALL(TARGETS frame_transforms
# 	EXPORT export_frame_transforms
# 	ARCHIVE DESTINATION lib
# 	LIBRARY DESTINATION lib
# 	RUNTIME DESTINATION bin
# 	INCLUDES DESTINATION include
# )

# # INSTALL LAUNCH FILES
# INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# # Install tests
# INSTALL(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

IF(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
ENDIF()
ament_package()
