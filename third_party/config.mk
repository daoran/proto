MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src
NUM_CPU := $(shell nproc)

YAMLCPP_TAG := "yaml-cpp-0.7.0"
SUITESPARSE_TAG := "v7.10.2"
EIGEN_TAG := "3.4.0"
CERES_TAG := "2.2.0"
ASSIMP_TAG := "v5.4.3"
GLFW_TAG := "3.4"
OPENCV_TAG := "4.10.0"
APRILTAG_TAG := "v3.4.3"
DBOW2_TAG := "v1.1-free"

export CLICOLOR=0

###############################################################################
# UTILS
###############################################################################

BUILD_YAMLCPP_CMD := \
	if [ -f $(PREFIX)/lib/libyaml-cpp.a ]; then return 0; fi \
		&& cd "$(SRC_PATH)/yaml-cpp" \
		&& git checkout $(YAMLCPP_TAG) \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

BUILD_STB_CMD := \
	cd "$(PREFIX)/include" \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_c_lexer.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_connected_components.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_divide.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_ds.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_dxt.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_easy_font.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_herringbone_wang_tile.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_hexwave.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image_resize.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image_write.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_include.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_leakcheck.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_perlin.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_rect_pack.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_sprintf.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_textedit.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_tilemap_editor.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_truetype.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_voxel_render.h;

BUILD_LZ4_CMD := \
	if [ -f $(PREFIX)/lib/liblz4.a ]; then return 0; fi \
		&& cd "$(SRC_PATH)/lz4" \
		&& make install PREFIX="$(PREFIX)";

###############################################################################
# LINEAR ALGEBRA
###############################################################################

BUILD_SUITESPARSE_CMD := \
	if [ -f $(PREFIX)/lib/libsuitesparseconfig.so ]; then return 0; fi \
		&& cd "$(SRC_PATH)/SuiteSparse" \
		&& git checkout $(SUITESPARSE_TAG) \
		&& if [ -f $(PREFIX)/lib/libsuitesparseconfig.a ]; then return 0; fi \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

BUILD_EIGEN_CMD := \
	if [ -f $(PREFIX)/include/eigen3/Eigen/Core ]; then return 0; fi \
		&& cd "$(SRC_PATH)/eigen" \
		&& git checkout $(EIGEN_TAG) \
		&& if [ -f $(PREFIX)/include/eigen3/Eigen/Core ]; then return 0; fi \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

BUILD_CERES_CMD := \
	if [ -f $(PREFIX)/lib/libceres.a ]; then return 0; fi \
		&& cd "$(SRC_PATH)/ceres-solver" \
		&& git checkout $(CERES_TAG) \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

###############################################################################
# COMPUTER VISION
###############################################################################

BUILD_OPENCV_CMD := \
	if [ -f $(PREFIX)/lib/libopencv_core.so ]; then return 0; fi \
		&& cd "$(SRC_PATH)/opencv" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

BUILD_APRILTAG_CMD := \
	if [ -f $(PREFIX)/lib/libapriltag.so ]; then return 0; fi \
		&& cd "$(SRC_PATH)/apriltag" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

###############################################################################
# COMPUTER GRAPHICS
###############################################################################

BUILD_ASSIMP_CMD := \
	if [ -f $(PREFIX)/lib/libassimp.so ]; then return 0; fi \
		&& cd "$(SRC_PATH)/assimp" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;

BUILD_KHR_CMD := \
	if [ -f $(PREFIX)/include/KHR/khrplatform.h ]; then return 0; fi \
		&& cd "$(SRC_PATH)/KHR" \
		&& mkdir -p "$(PREFIX)/include/KHR" \
		&& ln -sf $(SRC_PATH)/KHR/khrplatform.h "$(PREFIX)/include/KHR";

BUILD_GLAD_CMD := \
	if [ -f $(PREFIX)/lib/libglad.a ]; then return 0; fi \
		&& cd "$(SRC_PATH)/glad" \
		&& mkdir -p "$(PREFIX)/include/glad" \
		&& ln -sf $(SRC_PATH)/glad/glad.h "$(PREFIX)/include/glad" \
		&& gcc -I$(PREFIX)/include -c glad.c -o glad.o \
		&& ar rcs libglad.a glad.o \
		&& ln -sf $(SRC_PATH)/glad/libglad.a "$(PREFIX)/lib/libglad.a";

BUILD_GLFW_CMD := \
	if [ -f $(PREFIX)/lib/libglfw3.a ]; then return 0; fi \
		&& cd "$(SRC_PATH)/glfw" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH="$(PREFIX)" \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make -j$(NUM_CPU) \
		&& make install;
