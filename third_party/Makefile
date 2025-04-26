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
	@rm -rf "$(SRC_PATH)/assimp/build"
	@rm -rf "$(SRC_PATH)/glad/build"
	@rm -rf "$(SRC_PATH)/glfw/build"

install: setup apt_pkgs build_deps ## Install all dependencies

apt_pkgs:  ## Install apt packages
	@sudo apt-get update -qqq
	@sudo apt-get install -y -qqq \
		dialog \
		apt-utils \
		git \
		mercurial \
		cmake \
		g++ \
		clang \
		tcc
	@sudo apt-get install -y -qqq liblapacke liblapacke-dev
	@sudo apt-get install -y -qqq libomp-dev

build_deps: setup ## Build dependencies
	@# utils
	make -s build_yamlcpp
	make -s build_stb
	make -s build_lz4
	@# linear algebra
	make -s build_gklib
	make -s build_metis
	make -s build_suitesparse
	make -s build_eigen
	make -s build_ceres
	@# computer vision
	make -s build_opencv
	make -s build_apriltag
	make -s build_dbow2
	make -s build_fbow
	make -s build_flann
	@# computer graphics
	make -s build_assimp
	make -s build_khr
	make -s build_glad
	make -s build_glfw

###############################################################################
# UTILS
###############################################################################

build_yamlcpp:  ## Build yaml-cpp
	@{ ${BUILD_YAMLCPP_CMD} } > $(PREFIX)/log/yamlcpp.log 2>&1

build_stb:  ## Build stb
	@{ $(BUILD_STB_CMD) } > $(PREFIX)/log/stb.log 2>&1

build_lz4:  ## Build lz4
	@{ $(BUILD_LZ4_CMD) } > $(PREFIX)/log/lz4.log 2>&1

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

###############################################################################
# COMPUTER VISION
###############################################################################

build_opencv: build_eigen build_suitesparse ## Build opencv
	@echo "building opencv ..."
	@{ $(BUILD_OPENCV_CMD) } > $(PREFIX)/log/opencv.log 2>&1

build_apriltag: build_opencv ## Build apriltag
	@echo "building apriltag ..."
	@{ $(BUILD_APRILTAG_CMD) } > $(PREFIX)/log/apriltag.log 2>&1

build_dbow2:  ## Build dbow2
	@echo "building dbow2 ..."
	@{ $(BUILD_DBOW2_CMD) } > $(PREFIX)/log/dbow2.log 2>&1

build_fbow:  ## Build fbow
	@echo "building fbow ..."
	@{ $(BUILD_FBOW_CMD) } > $(PREFIX)/log/fbow.log 2>&1

build_flann: build_lz4  ## Build flann
	@echo "building flann ..."
	@{ $(BUILD_FLANN_CMD) } > $(PREFIX)/log/flann.log 2>&1

###############################################################################
# COMPUTER GRAPHICS
###############################################################################

build_assimp: ## Build assimp
	@{ $(BUILD_ASSIMP_CMD) } > $(PREFIX)/log/assimp.log 2>&1

build_khr:  ## Build khr
	@{ $(BUILD_KHR_CMD) } > $(PREFIX)/log/khr.log 2>&1

build_glad: build_khr  ## Build glad
	@{ $(BUILD_GLAD_CMD) } > $(PREFIX)/log/glad.log 2>&1

build_glfw: build_glad  ## Build glfw
	@{ $(BUILD_GLFW_CMD) } > $(PREFIX)/log/glfw.log 2>&1
