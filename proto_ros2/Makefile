DATA_DIR=/data

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-15s\033[0m %s\n", $$1, $$2}'

install_docker: ## Install Docker (Ubuntu only)
	@bash scripts/install_docker.bash

build:  ## Build proto_ros2
	@cd docker && \
		docker build -t proto_ros2 \
			--build-arg USERNAME=${USER} \
			--build-arg USER_ID=`id -u` \
			--build-arg GROUP_ID=`id -g` .

run: build   ## Run proto_ros2
	@xhost +local:docker
	@cd docker && \
		docker run -e DISPLAY \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
			-v /tmp:/tmp \
			-v $(DATA_DIR):$(DATA_DIR) \
			--network="host" \
			-it --rm proto_ros2 /bin/bash
