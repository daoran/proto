MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))
CATKIN_WS=~/catkin_ws

.PHONY: third_party docs build install tests ci run_test_xyz clean

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
	@pip3 install sphinx
	@pip3 install sphinx-autobuild
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@rm -rf docs/build
	@sphinx-autobuild docs/source docs/build/html

build: ## Build libxyz
	@cd xyz && make -s libxyz

install: build ## Install libxyz
	@cd xyz && make -s install

tests: ## Run unittests
	@cd xyz && make -s tests

ci: ## Run CI tests
	@cd xyz && make -s ci

clean: ## Clean
	@cd xyz && make -s clean
