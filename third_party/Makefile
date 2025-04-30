MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src

help:
	@echo "make targets:"
	@echo "----------------------------------------"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' Makefile \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "%-20s%s\n", $$1, $$2}'

build: ## Build
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@bash build_deps.bash

clean: ## Clean
	@rm -rf bin
	@rm -rf cmake
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share
	@rm -rf "$(SRC_PATH)/SuiteSparse"
	@rm -rf "$(SRC_PATH)/apriltag-imgs"
	@rm -rf "$(SRC_PATH)/apriltag"
	@rm -rf "$(SRC_PATH)/assimp"
	@rm -rf "$(SRC_PATH)/ceres-solver"
	@rm -rf "$(SRC_PATH)/eigen.git"
	@rm -rf "$(SRC_PATH)/gflags"
	@rm -rf "$(SRC_PATH)/glfw"
	@rm -rf "$(SRC_PATH)/glog"
	@rm -rf "$(SRC_PATH)/lz4"
	@rm -rf "$(SRC_PATH)/opencv"
	@rm -rf "$(SRC_PATH)/stb"
	@rm -rf "$(SRC_PATH)/yaml-cpp"
