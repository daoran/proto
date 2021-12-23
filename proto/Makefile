include config.mk
.PHONY: lib python

lib:
	@make -C lib -s

run_tests:
	@cd ./build/bin/ && ./test_proto
	# @cd python && python3 proto.py

clean:
	@rm -rf ${BLD_DIR}
	@echo "[./$(shell realpath --relative-to ${PWD}/.. ${BLD_DIR})] cleaned!"
