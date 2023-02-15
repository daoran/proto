MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
CATKIN_WS=~/catkin_ws

.PHONY: third_party docs proto ros

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-12s\033[0m%s\n", $$1, $$2}'

third_party: ## Install dependencies
	@git submodule init
	@git submodule update
	@make -s -C third_party

docs: ## Build docs
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@rm -rf docs/build
	@sphinx-autobuild docs/source docs/build/html

build: ## Build libproto
	@cd proto && make -s libproto

tests: ## Run unittests
	@cd proto && make -s tests

run_test_proto:
	@cd proto && make -s run_test_proto

clean: ## Clean
	@cd proto && make -s clean

# ros: ## Build proto_ros
# 	@mkdir -p ${CATKIN_WS}/src
# 	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/proto_ros . \
# 		&& . /opt/ros/melodic/setup.sh \
# 		&& cd .. \
# 		&& catkin build proto_ros -DCMAKE_BUILD_TYPE=RELEASE -j2
