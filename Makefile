MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
CATKIN_WS=~/catkin_ws

.PHONY: default deps debug release install ros format_code docs

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| sort \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-15s\033[0m%s\n", $$1, $$2}'

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

clean: ## Clean build
	@cd proto && make -s clean

debug: ## Build in debug mode
	@cd proto && make -s -DDEBUG

build: ## Build in release mode
	@cd proto && make -s lib

run_tests: release  ## Run unit-tests
	@cd proto && make -s run_tests

install: ## Install proto
	@if [ ! -d proto/build ]; then \
		echo "Error: Not built yet!"; \
		exit 1; \
	else \
		cd proto; \
		make -s install; \
	fi

ros: ## Build proto_ros
	@mkdir -p ${CATKIN_WS}/src
	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/proto_ros . \
		&& . /opt/ros/melodic/setup.sh \
		&& cd .. \
		&& catkin build proto_ros -DCMAKE_BUILD_TYPE=RELEASE -j2

format_code: ## Format code
	@bash ./scripts/format_code.bash

docs: ## Build docs
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@sphinx-autobuild docs/source docs/build/html
