default:

# usage:
# 	@cat<<-EOF
# 		debug:
# 			build prototype in debug mode
#
# 		release:
# 			build prototype in release mode
# 	EOF

deps:
	@git submodule init
	@git submodule update

debug: deps
	@mkdir -p build
	@cd build && cmake -DCMAKE_BUILD_TYPE=DEBUG .. && make -s

release: deps
	@mkdir -p build
	@cd build && cmake -DCMAKE_BUILD_TYPE=RELEASE .. && make -s

install:
	@cd build && make -s install
