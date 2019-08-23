CATKIN_WS=${HOME}/catkin_ws
CATKIN_WS_SRC=${HOME}/catkin_ws/src
PROTO_ROS=${CATKIN_WS_SRC}/proto_ros

PREFIX=/usr/local
LIB_PROTO=${PREFIX}/lib/libproto.so
PROTO_REPO=https://github.com/chutsu/proto
PROTO_ROS_REPO=https://github.com/chutsu/proto_ros

default: release

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS_SRC}

${PROTO_ROS}: ${CATKIN_WS}
	echo "${PROTO_ROS}"
	@cd ${CATKIN_WS_SRC} && git clone ${PROTO_ROS_REPO}
	@cd ${CATKIN_WS_SRC} && catkin build proto_ros

${LIB_PROTO}:
	@cd ${PREFIX}/src \
		&& sudo git -C proto pull || sudo git clone ${PROTO_REPO} \
		&& cd ${PREFIX}/src/proto \
		&& sudo make install_deps \
		&& sudo make release \
		&& sudo make install

debug: ${PROTO_ROS} ${LIB_PROTO}
	@cd ${CATKIN_WS} && catkin build -DCMAKE_BUILD_TYPE=DEBUG

release: ${PROTO_ROS} ${LIB_PROTO}
	@cd ${CATKIN_WS} && catkin build -DCMAKE_BUILD_TYPE=RELEASE
