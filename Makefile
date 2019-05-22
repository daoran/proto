include config.mk

define usage
[TARGETS]:
  deps:
    Clone prototype dependencies

  debug:
    Build prototype in debug mode

  release:
    Build prototype in release mode

  install:
    Install prototype

  install_deps:
    Install prototype dependencies

  format_code:
    Format prototype code using clang-format

  docs:
    Generate docs for prototype
endef
export usage

default:
	@echo "$$usage"

deps:
	@git submodule init
	@git submodule update

build_dir:
	@mkdir -p build

debug: deps build_dir
	@cd build && cmake -DCMAKE_BUILD_TYPE=DEBUG .. && make -s

release: deps build_dir
	@cd build && cmake -DCMAKE_BUILD_TYPE=RELEASE .. && make -s

install:
	@cd build && make -s install

install_deps:
	@bash ./scripts/deps/install.bash

format_code:
	@bash ./scripts/format_code.bash

docs:
	@cd scripts/api && python3 api.py

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS_SRC}

${CATKIN_WS_SRC}/prototype_ros: ${CATKIN_WS}
	@cd ${CATKIN_WS_SRC} && git clone ${PROTOTYPE_ROS_REPO}
	@cd ${CATKIN_WS_SRC} && catkin build prototype_ros
