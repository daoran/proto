MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)/deps
NUM_PROCS := $(shell expr `nproc` / 2)

define cmake_build
	cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=$1 \
			-DCMAKE_PREFIX_PATH=$(PREFIX) \
			-DCMAKE_LIBRARY_PATH=$(PREFIX)\lib \
			.. \
		&& make -j$(NUM_PROCS)
endef

.PHONY: help
help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-10s\033[0m%s\n", $$1, $$2}'

.PHONY: setup
setup:
	@mkdir -p build

.PHONY: all
all: deps release ## Build all

.PHONY: deps
deps: setup ## Build dependencies
	@echo "[Build Dependencies]"
	@make -s -C deps all

.PHONY: release
release: setup ## Build in release mode
	$(call cmake_build,RelWithDebInfo)

.PHONY: debug
debug: setup ## Build in debug mode
	$(call cmake_build,Debug)

#.PHONY: docs
#docs: ## Build docs
#	ctags -R docs/source/notes --languages=reStructuredText
#	make -s -C docs html
