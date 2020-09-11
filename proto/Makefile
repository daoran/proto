include config.mk
NB_JOBS=4

default: dirs libproto tests
.PHONY: dirs libproto tests

dirs:
	@mkdir -p $(BLD_DIR)

clean:
	@echo "cleaning build files for libproto ..."
	@rm -rf $(BLD_DIR)
	@echo "done!"

libproto:
	@echo "building [libproto]"
	@make -s -C lib -j$(NB_JOBS)

tests:
	@echo "building [unit-tests]"
	@make -s -C tests -j$(NB_JOBS)
