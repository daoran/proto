MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
CATKIN_WS=~/catkin_ws

.PHONY: deps docs ros

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-15s\033[0m%s\n", $$1, $$2}'

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

docs: ## Build docs
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@sphinx-autobuild docs/source docs/build/html

# ros: ## Build proto_ros
# 	@mkdir -p ${CATKIN_WS}/src
# 	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/proto_ros . \
# 		&& . /opt/ros/melodic/setup.sh \
# 		&& cd .. \
# 		&& catkin build proto_ros -DCMAKE_BUILD_TYPE=RELEASE -j2
