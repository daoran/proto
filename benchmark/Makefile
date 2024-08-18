DATA_DIR=/data
HOME_PATH=/home/slam_bench

BUILD_TARGETS := build_kalibr
BUILD_TARGETS += build_msckf
BUILD_TARGETS += build_okvis
BUILD_TARGETS += build_orbslam3
BUILD_TARGETS += build_vins_mono
BUILD_TARGETS += build_vins_mono
BUILD_TARGETS += build_vins_fusion

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-30s\033[0m %s\n", $$1, $$2}'

install_docker: ## Install Docker
	@bash scripts/install_docker.bash

build_all: $(BUILD_TARGETS) ## Build all

build_kalibr:  ## Build Kalibr
	@cd kalibr && docker build -t benchmark/kalibr \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

build_msckf:  ## Build MSCKF-VIO
	@cd msckf_vio && docker build -t benchmark/msckf \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

build_okvis:  ## Build OKVIS
	@cd okvis && docker build -t benchmark/okvis \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

build_orbslam3: ## Build ORBSLAM3
	@cd orbslam3 && docker build -t benchmark/orbslam3 \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

build_vins_mono: ## Build VINS-Mono
	@cd vins-mono && docker build -t benchmark/vins-mono \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

build_vins_fusion: ## Build VINS-Fusion
	@cd vins-fusion && docker build -t benchmark/vins-fusion \
		--build-arg USER_ID=`id -u` \
		--build-arg GROUP_ID=`id -g` .

run_kalibr: build_kalibr  ## Run Kalibr
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/kalibr /bin/bash

run_msckf: build_msckf  ## Run MSKCF
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/msckf /bin/bash

run_okvis: build_okvis ## RUN OKVIS
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/okvis /bin/bash

run_orbslam3: build_orbslam3  ## Run ORBSLAM3
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/orbslam3 /bin/bash

run_vins_mono: build_vins_mono  ## Run VINS-Mono
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/vins-mono /bin/bash

run_vins_fusion: build_vins_fusion  ## Run VINS-Fusion
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD)/configs:/home/docker/configs \
		--network="host" \
		-it --rm benchmark/vins-fusion /bin/bash
