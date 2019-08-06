MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
BUILD_DIR=${PROJ_PATH}/build


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
    Generate docs for proto.
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

format_code:
	@bash ./scripts/format_code.bash

docs:
	@cd scripts/api && python3 api.py
