MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
BUILD_DIR=${PROJ_PATH}/proto/build
CATKIN_WS=~/catkin_ws

.PHONY: default deps debug release install ros format_code docs

define usage
[TARGETS]:
  deps:
    Install proto dependencies.

  debug:
    Build proto in debug mode.

  release:
    Build proto in release mode.

  install:
    Install proto to '$$PREFIX'. By default this is "/usr/local".

  format_code:
    Format proto code using clang-format.

  docs:
    Start a webserver to write docs.
endef
export usage

default:
	@echo "$$usage"

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

${BUILD_DIR}:
	@mkdir -p ${BUILD_DIR}
	# @make -s deps

debug: ${BUILD_DIR}
	@cd proto/build && cmake -DCMAKE_BUILD_TYPE=DEBUG .. && make -s -j2

release: ${BUILD_DIR}
	@cd proto/build && cmake -DCMAKE_BUILD_TYPE=RELEASE .. && make -s -j2

install:
	@if [ ! -d proto/build ]; then \
		echo "Error: Not built yet!"; \
		exit 1; \
	else \
		cd proto/build && make -s install; \
	fi

ros:
	@mkdir -p ${CATKIN_WS}/src
	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/proto_ros . \
		&& . /opt/ros/melodic/setup.sh \
		&& cd .. \
		&& catkin build proto_ros -DCMAKE_BUILD_TYPE=RELEASE -j2

format_code:
	@bash ./scripts/format_code.bash

docs:
	# @cd docs && xdg-open http://127.0.0.1:8000 && python -m SimpleHTTPServer 8000
	@cd docs && xdg-open http://127.0.0.1:35729 && livereload
