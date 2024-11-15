# PATHS
MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
CUR_DIR := $(shell pwd)
BLD_DIR := build
INC_DIR := $(realpath $(PWD))
DEPS_DIR := $(realpath third_party)
PREFIX := /opt/xyz
PYTHON3_PATH := $(shell python3 -c "import site; print(site.getsitepackages()[0])")

# COMPILER SETTINGS
BUILD_TYPE := debug
# BUILD_TYPE := release
ADDRESS_SANITIZER := 1
# ADDRESS_SANITIZER := 0
# CC := clang
CC := gcc
# CC := tcc


# LIBRARIES
STB_CFLAGS:=-I$(DEPS_DIR)/src/stb
OPENSSL_LDFLAGS := -lssl -lcrypto
GLEW_LDFLAGS := -lGLEW
SDL2_CFLAGS:=$(shell sdl2-config --cflags)
SDL2_LDFLAGS := $(shell sdl2-config --libs) -lSDL2_image
OPENGL_LDFLAGS := $(SDL2_LDFLAGS) $(GLEW_LDFLAGS) -lGL
BLAS_LDFLAGS := -lblas -llapack -llapacke
SUITESPARSE_LDFLAGS := -llapack -lcamd -lamd -lccolamd -lcolamd -lcholmod -lcxsparse
CERES_CFLAGS := -I/usr/include/eigen3
CERES_LDFLAGS := -lgflags -lglog -lceres
ASSIMP_LDFLAGS := -lassimp
APRILTAG_LDFLAGS := -L$(DEPS_DIR)/lib -lapriltag
YAML_LDFLAGS := -lyaml
XYZ_LDFLAGS := -L$(BLD_DIR) -lxyz


# CFLAGS
CFLAGS := -Wall -Wpedantic -Wstrict-prototypes

ifeq ($(BUILD_TYPE), debug)
	CFLAGS += -g -fopenmp
else
	CFLAGS += -g -O2 -march=native -mtune=native -DNDEBUG -fopenmp
endif

ifeq ($(ADDRESS_SANITIZER), 1)
ifeq ($(CC), gcc)
  CFLAGS += -fsanitize=address -static-libasan
else
  CFLAGS += -fsanitize=address -static-libsan
endif
endif

CFLAGS += \
	-I$(INC_DIR) \
	-I$(DEPS_DIR)/include \
	-fPIC \
	$(SDL2_CFLAGS) \
	$(STB_CFLAGS) \
	$(CERES_CFLAGS)


# LDFLAGS
RPATH := -Wl,-rpath,$(DEPS_DIR)/lib
LDFLAGS= \
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
	-lglfw3 \
	-lstdc++ \
	-lpthread \
  -lm


# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs


# TARGETS
LIBXYZ := $(BLD_DIR)/libxyz.a
LIBXYZ_OBJS := \
	$(BLD_DIR)/xyz.o \
	$(BLD_DIR)/aprilgrid.o \
	$(BLD_DIR)/euroc.o \
	$(BLD_DIR)/gui.o \
	$(BLD_DIR)/sbgc.o \
	$(BLD_DIR)/ceres_bridge.o

TESTS := \
	test_xyz \
	test_aprilgrid \
	test_sbgc \
	test_ubx
