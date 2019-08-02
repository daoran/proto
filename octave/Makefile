all: dataset config
	@echo "Done!"

deps:
	pkg install -forge quaternion;
	pkg install -forge geometry;
	pkg install -forge symbolic;

dataset:
	@make -s -C proto/dataset

config:
	@make -s -C proto/core/config

clean:
	@make -s -C proto/dataset clean
	@make -s -C proto/core/config clean
