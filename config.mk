# PATHS
MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
CUR_DIR := $(shell pwd)
BLD_DIR := build
INC_DIR := $(realpath $(PWD))
THIRD_PARTY_DIR := $(realpath third_party)
PREFIX := /opt/xyz
PYTHON3_PATH := $(shell python3 -c "import site; print(site.getsitepackages()[0])")
export LD_LIBRARY_PATH:=$(LD_LIBRARY_PATH):$(THIRD_PARTY_DIR)/lib

# SETTINGS
BUILD_TYPE := debug
# BUILD_TYPE := release
CI_MODE := 0
CC := clang
# CC := gcc
# CC := tcc


# LIBRARIES
STB_CFLAGS:=-I$(THIRD_PARTY_DIR)/src/stb
OPENSSL_LDFLAGS := -lssl -lcrypto
GLAD_LDFLAGS := -lglad
OPENGL_LDFLAGS := $(GLAD_LDFLAGS) -lGL
FREETYPE_CFLAGS := $(shell pkg-config --cflags freetype2)
FREETYPE_LDFLAGS := $(shell pkg-config --libs freetype2)
BLAS_LDFLAGS := -lblas -llapack -llapacke
SUITESPARSE_LDFLAGS := -llapack -lcamd -lamd -lccolamd -lcolamd -lcholmod -lcxsparse
CERES_CFLAGS := -I$(THIRD_PARTY_DIR)/include/eigen3
CERES_LDFLAGS := -lgflags -lglog -lceres
ASSIMP_LDFLAGS := -lassimp
APRILTAG_LDFLAGS := -lapriltag
YAML_LDFLAGS := -lyaml
XYZ_LDFLAGS := -L$(BLD_DIR) -lxyz


# CFLAGS
CFLAGS := -std=c99 -D_POSIX_C_SOURCE=200809L -D_GNU_SOURCE -Wall -Wpedantic -Wstrict-prototypes

ifeq ($(BUILD_TYPE), debug)
	CFLAGS += -g -fopenmp
	ADDRESS_SANITIZER := 1
else
	CFLAGS += -g -O2 -march=native -mtune=native -DNDEBUG -fopenmp
	ADDRESS_SANITIZER := 0
endif

ifeq ($(ADDRESS_SANITIZER), 1)
ifeq ($(CC), gcc)
	CFLAGS += -fsanitize=address -static-libasan
else
	CFLAGS += -fsanitize=address -static-libsan
endif
endif

ifeq ($(CI_MODE), 1)
	CFLAGS += -DMU_REDIRECT_STREAMS=1 -DCI_MODE=1
endif


CFLAGS += \
	-I$(INC_DIR) \
	-I$(THIRD_PARTY_DIR)/include \
	-fPIC \
	$(STB_CFLAGS) \
	$(FREETYPE_CFLAGS) \
	$(CERES_CFLAGS)


# LDFLAGS
RPATH := -Wl,-rpath,$(THIRD_PARTY_DIR)/lib
LDFLAGS= \
	-L$(THIRD_PARTY_DIR)/lib \
	$(RPATH) \
	$(XYZ_LDFLAGS) \
	$(CERES_LDFLAGS) \
	$(APRILTAG_LDFLAGS) \
	$(OPENGL_LDFLAGS) \
	$(SUITESPARSE_LDFLAGS) \
	$(BLAS_LDFLAGS) \
	$(OPENSSL_LDFLAGS) \
	$(ASSIMP_LDFLAGS) \
	$(YAML_LDFLAGS) \
	$(FREETYPE_LDFLAGS) \
	-lglfw \
	-lstdc++ \
	-lpthread \
	-lm \
	-ldl


# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs


# TARGETS
LIBXYZ := $(BLD_DIR)/libxyz.a
LIBXYZ_OBJS := \
	$(BLD_DIR)/xyz.o \
	$(BLD_DIR)/xyz_http.o \
	$(BLD_DIR)/xyz_kitti.o \
	$(BLD_DIR)/xyz_gnuplot.o \
	$(BLD_DIR)/xyz_aprilgrid.o \
	$(BLD_DIR)/xyz_cv.o \
	$(BLD_DIR)/xyz_control.o \
	$(BLD_DIR)/xyz_gimbal.o \
	$(BLD_DIR)/xyz_mav.o \
	$(BLD_DIR)/xyz_euroc.o \
	$(BLD_DIR)/xyz_octree.o \
	$(BLD_DIR)/xyz_gui.o \
	$(BLD_DIR)/xyz_voxel.o \
	$(BLD_DIR)/xyz_kdtree.o \
	$(BLD_DIR)/xyz_se.o


# TESTS
TESTS := \
	$(BLD_DIR)/test_aprilgrid \
	$(BLD_DIR)/test_control \
	$(BLD_DIR)/test_cv \
	$(BLD_DIR)/test_euroc \
	$(BLD_DIR)/test_gimbal \
	$(BLD_DIR)/test_gnuplot \
	$(BLD_DIR)/test_gui \
	$(BLD_DIR)/test_http \
	$(BLD_DIR)/test_kitti \
	$(BLD_DIR)/test_mav \
	$(BLD_DIR)/test_octree \
	$(BLD_DIR)/test_voxel \
	$(BLD_DIR)/test_xyz \
	$(BLD_DIR)/test_kdtree \
	$(BLD_DIR)/test_se
