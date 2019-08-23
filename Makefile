MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
BUILD_DIR=${PROJ_PATH}/build
CATKIN_WS=~/catkin_ws

.PHONY: ros

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

  notes:
    Generate notes for proto.
endef
export usage

default:
	@echo "$$usage"

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

${BUILD_DIR}:
	@mkdir -p ${BUILD_DIR}
	@make -s deps

debug: ${BUILD_DIR}
	@cd build && cmake -DCMAKE_BUILD_TYPE=DEBUG .. && make -s

release: ${BUILD_DIR}
	@cd build && cmake -DCMAKE_BUILD_TYPE=RELEASE .. && make -s

install:
	@if [ ! -d build ]; then \
		echo "Error: Not built yet!"; \
		exit 1; \
	else \
		cd build && make -s install; \
	fi

ros:
	@cd ${CATKIN_WS} && catkin build proto_ros

format_code:
	@bash ./scripts/format_code.bash

notes:
	@python3 ./scripts/notes/notes.py
