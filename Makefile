MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
CATKIN_WS=~/catkin_ws

.PHONY: deps docs proto ros

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-7s\033[0m%s\n", $$1, $$2}'

deps: ## Install dependencies
	@make -s -C deps

docs: ## Build docs
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@sphinx-autobuild docs/source docs/build/html

build: ## Build libproto
	@cd proto && make -s

tests: ## Run unittests
	@cd proto && make -s tests

clean: ## Clean
	@cd proto && make -s clean

# ros: ## Build proto_ros
# 	@mkdir -p ${CATKIN_WS}/src
# 	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/proto_ros . \
# 		&& . /opt/ros/melodic/setup.sh \
# 		&& cd .. \
# 		&& catkin build proto_ros -DCMAKE_BUILD_TYPE=RELEASE -j2
