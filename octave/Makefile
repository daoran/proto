all: dataset config
	@echo "Done!"

deps:
	pkg install -forge quaternion;
	pkg install -forge geometry;
	pkg install -forge symbolic;

dataset:
	@make -s -C prototype/dataset

config:
	@make -s -C prototype/core/config

clean:
	@make -s -C prototype/dataset clean
	@make -s -C prototype/core/config clean
