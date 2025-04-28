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

setup:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share

clean: ## Clean
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share

distclean:
	@# utils
	@rm -rf "$(SRC_PATH)/yaml-cpp/build"
	@rm -rf "$(SRC_PATH)/gflags/build"
	@rm -rf "$(SRC_PATH)/glog/build"
	@# linear algebra
	@rm -rf "$(SRC_PATH)/SuiteSparse/build"
	@rm -rf "$(SRC_PATH)/eigen/build"
	@rm -rf "$(SRC_PATH)/ceres-solver/build"
	@# computer vision
	@rm -rf "$(SRC_PATH)/opencv/build"
	@rm -rf "$(SRC_PATH)/apriltag/build"
	@# computer graphics
	@rm -rf "$(SRC_PATH)/glfw/build"
	@rm -rf "$(SRC_PATH)/assimp/build"

install: setup apt_pkgs build_deps ## Install all dependencies
