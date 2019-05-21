default: usage

usage:
	@echo "debug"
	@echo "release"

deps:
	@git submodule init
	@git submodule update

debug: deps
	@mkdir -p build
	@cd build && cmake .. && make -s

release: deps
	@mkdir -p build
	@cd build && cmake .. && make -s
