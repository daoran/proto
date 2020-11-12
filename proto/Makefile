include config.mk
NB_JOBS=4

default: dirs libproto tests
.PHONY: dirs libproto tests

dirs:
	@mkdir -p $(BLD_DIR)

clean:
	@echo "Cleaning build files for libproto ..."
	@rm -rf $(BLD_DIR)
	@echo "Done!"

libproto:
	@echo "Building [libproto]"
	@make -s -C lib -j$(NB_JOBS)

tests:
	@echo "Building [unit-tests]"
	@make -s -C tests -j$(NB_JOBS)

install:
	@echo "Installing [libproto]"
	@mkdir -p /usr/local/include/proto
	@cp -rv lib/*.hpp /usr/local/include/proto
	@cp -v build/libproto.a /usr/local/lib
	@echo "Done!"
