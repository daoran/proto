include config.mk

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
	@# utils
	@rm -rf "$(SRC_PATH)/yamlcpp/build"
	@# linear algebra
	@rm -rf "$(SRC_PATH)/GKlib/build"
	@rm -rf "$(SRC_PATH)/SuiteSparse/build"
	@rm -rf "$(SRC_PATH)/eigen/build"
	@rm -rf "$(SRC_PATH)/ceres-solver/build"
	@# computer vision
	@rm -rf "$(SRC_PATH)/opencv/build"
	@rm -rf "$(SRC_PATH)/apriltag/build"
	@# computer graphics
	@rm -rf "$(SRC_PATH)/glad/build"
	@rm -rf "$(SRC_PATH)/glfw/build"
	@rm -rf "$(SRC_PATH)/assimp/build"

install: setup apt_pkgs build_deps ## Install all dependencies

apt_pkgs:  ## Install apt packages
	@sudo apt-get update -qqq
	@sudo apt-get install -y -qqq \
		dialog \
		apt-utils \
		build-essential \
		pkg-config \
		make \
		cmake \
		git \
		mercurial \
		g++ \
		clang \
		tcc \
		libx11-dev \
		libwayland-dev \
		libxkbcommon-dev \
		libxrandr-dev \
		libxinerama-dev \
		libxcursor-dev \
		libxi-dev
	@sudo apt-get install -y -qqq \
		libomp-dev \
		libmpfr-dev \
		libblas-dev \
		liblapack-dev \
		liblapacke-dev \
		libmetis-dev

# clone_repos:
# 	@# utils
# 	git clone -b $(YAMLCPP_TAG), https://github.com/jbeder/yaml-cpp
# 	git clone -b $(STB_TAG), https://github.com/nothings/stb
# 	git clone -b $(LZ4_TAG), https://github.com/lz4/lz4
# 	git clone -b $(GFLAGS_TAG), https://github.com/gflags/gflags
# 	git clone -b $(GLOG_TAG), https://github.com/google/glog
# 	@# linear algebra
# 	git clone -b $(SUITESPARSE_TAG), https://github.com/DrTimothyAldenDavis/SuiteSparse
# 	git clone -b $(EIGEN_TAG), https://gitlab.com/libeigen/eigen.git
# 	git clone -b $(CERES_TAG), https://github.com/ceres-solver/ceres-solver
# 	@# computer vision
# 	git clone -b $(OPENCV_TAG), https://github.com/opencv/opencv
# 	git clone -b $(APRILTAG_TAG), https://github.com/AprilRobotics/apriltag
# 	git clone -b "master", https://github.com/AprilRobotics/apriltag-imgs
# 	@# computer graphics
# 	git clone -b $(ASSIMP_TAG), https://github.com/assimp/assimp
# 	git clone -b $(GLFW_TAG), https://github.com/glfw/glfw

build_deps: setup clone_repos ## Build dependencies
	@make -s build_utils
	@make -s build_linalg
	@make -s build_cv
	@make -s build_cg

###############################################################################
# UTILS
###############################################################################

build_yamlcpp:
	@echo "building yamlcpp ..."
	@{ ${BUILD_YAMLCPP_CMD} } > $(PREFIX)/log/yamlcpp.log 2>&1

build_stb:
	@echo "building stb ..."
	@{ $(BUILD_STB_CMD) } > $(PREFIX)/log/stb.log 2>&1

build_lz4:
	@echo "building lz4 ..."
	@{ $(BUILD_LZ4_CMD) } > $(PREFIX)/log/lz4.log 2>&1

build_gflags:
	@echo "building gflags ..."
	@{ $(BUILD_GFLAGS_CMD) } > $(PREFIX)/log/gflags.log 2>&1

build_glog:
	@echo "building glog ..."
	@{ $(BUILD_GLOG_CMD) } > $(PREFIX)/log/glog.log 2>&1

build_utils: build_yamlcpp build_stb build_lz4 build_gflags build_glog

###############################################################################
# LINEAR ALGEBRA
###############################################################################

build_suitesparse: ## Build suitesparse
	@echo "building suitesparse ..."
	@{ $(BUILD_SUITESPARSE_CMD) } > $(PREFIX)/log/suitesparse.log 2>&1

build_eigen: build_suitesparse ## Build eigen
	@echo "building eigen ..."
	@{ $(BUILD_EIGEN_CMD) } > $(PREFIX)/log/eigen.log 2>&1

build_ceres: build_eigen build_suitesparse  ## Build ceres-solver
	@echo "building ceres ..."
	@{ $(BUILD_CERES_CMD) } > $(PREFIX)/log/ceres.log 2>&1

build_linalg: build_ceres

###############################################################################
# COMPUTER VISION
###############################################################################

build_opencv: build_eigen build_suitesparse ## Build opencv
	@echo "building opencv ..."
	@{ $(BUILD_OPENCV_CMD) } > $(PREFIX)/log/opencv.log 2>&1

build_apriltag: build_opencv ## Build apriltag
	@echo "building apriltag ..."
	@{ $(BUILD_APRILTAG_CMD) } > $(PREFIX)/log/apriltag.log 2>&1

build_cv: build_opencv build_apriltag

###############################################################################
# COMPUTER GRAPHICS
###############################################################################

build_khr:  ## Build khr
	@echo "building khr ..."
	@{ $(BUILD_KHR_CMD) } > $(PREFIX)/log/khr.log 2>&1

build_glad: build_khr  ## Build glad
	@echo "building glad ..."
	@{ $(BUILD_GLAD_CMD) } > $(PREFIX)/log/glad.log 2>&1

build_glfw: build_glad  ## Build glfw
	@echo "building glfw ..."
	@{ $(BUILD_GLFW_CMD) } > $(PREFIX)/log/glfw.log 2>&1

build_assimp: ## Build assimp
	@echo "building assimp ..."
	@{ $(BUILD_ASSIMP_CMD) } > $(PREFIX)/log/assimp.log 2>&1

build_cg: build_khr build_glad build_glfw build_assimp
