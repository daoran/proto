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

ubuntu_usb:  ## Create Ubuntu USB boot-stick
	@echo "[Create Ubuntu install USB stick]"
	@sudo usb-creator-gtk

setup_chrony_server:  ## Setup chrony server
	@sudo apt-get remove ntp ntpdate -qq
	@sudo apt-get install chrony -qq
	@sudo cp configs/chrony_server.conf /etc/chrony/chrony.conf
	@sudo systemctl restart chrony.service
	@sleep 2
	@sudo systemctl --no-pager status chrony.service
	chronyc sourcestats

setup_chrony_client:  ## Setup chrony client
	@sudo apt-get remove ntp ntpdate -qq
	@sudo apt-get install chrony -qq
	@sudo cp configs/chrony_client.conf /etc/chrony/chrony.conf
	@sudo systemctl restart chrony.service
	@sleep 2
	@sudo systemctl --no-pager status chrony.service
	chronyc sources
