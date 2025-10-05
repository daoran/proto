MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src
LOG_PATH := $(PREFIX)/log
NUM_PROCS := $(shell expr `nproc` / 2)

YAML_VERSION=0.8.0
ABSEIL_VERSION=20250814.1
GTEST_VERSION=v1.17.0
EIGEN_VERSION=3.4.0
SUITESPARSE_VERSION=v7.11.0
CERES_VERSION=2.2.0
OPENCV_VERSION=4.11.0
APRILTAG_VERSION=v3.4.5

YAML_REPO=https://github.com/jbeder/yaml-cpp
ABSEIL_REPO=https://github.com/abseil/abseil-cpp
GTEST_REPO=https://github.com/google/googletest
EIGEN_REPO=https://gitlab.com/libeigen/eigen
SUITESPARSE_REPO=https://github.com/DrTimothyAldenDavis/SuiteSparse
CERES_REPO=https://github.com/ceres-solver/ceres-solver
OPENCV_REPO=https://github.com/opencv/opencv
APRILTAG_REPO=https://github.com/AprilRobotics/apriltag

define get_repo_name
$(basename $(notdir $(patsubst %/,%,$(1))))
endef

define cmake_build
	@echo "Building [$(call get_repo_name,$(1))] ..."
	@mkdir -p $(SRC_PATH)
	@mkdir -p $(LOG_PATH)
	@( \
	if [ ! -d "$(SRC_PATH)/$(call get_repo_name,$(1))" ]; then \
		cd $(SRC_PATH) && git clone $(1); \
	fi \
	&& cd $(SRC_PATH)/$(call get_repo_name,$(1)) \
		&& git checkout $(2) \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_COLOR_MAKEFILE=OFF \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX=$(PREFIX) \
			.. \
		&& make -j$(NUM_PROCS) \
		&& make install; \
	) > "$(LOG_PATH)/$(call get_repo_name,$(1)).log" 2>&1
endef

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

default: setup
.PHONY: setup clean
.PHONY: build_yaml build_abseil build_gtest
.PHONY: build_eigen build_suitesparse
.PHONY: build_ceres build_opencv build_apriltag build_apriltags

setup:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

clean: ## Clean
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share
	@rm -rf src/SuiteSparse/build
	@rm -rf src/abseil-cpp/build
	@rm -rf src/apriltag/build
	@rm -rf src/apriltags/build
	@rm -rf src/ceres-solver/build
	@rm -rf src/eigen/build
	@rm -rf src/googletest/build
	@rm -rf src/opencv/build
	@rm -rf src/yaml-cpp/build

build_yaml:  ## Build yaml
	$(call cmake_build,$(YAML_REPO),$(YAML_VERSION))

build_abseil:  ## Build abseil
	$(call cmake_build,$(ABSEIL_REPO),$(ABSEIL_VERSION))

build_gtest:  ## Build GTest
	$(call cmake_build,$(GTEST_REPO),$(GTEST_VERSION))

build_eigen:  ## Build Eigen
	$(call cmake_build,$(EIGEN_REPO),$(EIGEN_VERSION))

build_suitesparse: build_eigen ## Build SuiteSparse
	$(call cmake_build,$(SUITESPARSE_REPO),$(SUITESPARSE_VERSION))

build_ceres: build_abseil build_gtest build_suite_sparse ## Build ceres-solver
	$(call cmake_build,$(CERES_REPO),$(CERES_VERSION))

build_opencv: build_eigen ## Build OpenCV
	$(call cmake_build,opencv,$(OPENCV_REPO),$(OPENCV_VERSION))

build_apriltag: build_opencv  ## Build AprilTag
	$(call cmake_build,$(APRILTAG_REPO),$(APRILTAG_VERSION))

BUILD_TARGETS := \
	build_yaml \
	build_abseil \
	build_gtest \
	build_eigen \
	build_suitesparse \
	build_ceres \
	build_opencv \
	build_apriltag
all: $(BUILD_TARGETS)  ## Build all
