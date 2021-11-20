default: dataset config
	@echo "Done!"

.PHONY: deps dataset config tests clean

deps:
	pkg install -forge quaternion;
	pkg install -forge geometry;
	pkg install -forge symbolic;

dataset:
	@make -s -C proto/dataset

config:
	@make -s -C proto/core/config

tests:
	@./scripts/run_tests.bash

clean:
	@make -s -C proto/dataset clean
	@make -s -C proto/core/config clean
