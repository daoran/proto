BUILD_DIR=${PWD}/build

define usage
[TARGETS]:
  deps:
    Install prototype dependencies.

  debug:
    Build prototype in debug mode.

  release:
    Build prototype in release mode.

  install:
    Install prototype to $PREFIX. By default this is "/usr/local".

  format_code:
    Format prototype code using clang-format.

  docs:
    Generate docs for prototype.
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

install: release
	@cd build && make -s install

debug_install: debug
	@cd build && make -s install

format_code:
	@bash ./scripts/format_code.bash

docs:
	@cd scripts/api && python3 api.py
