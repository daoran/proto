# PATHS
BLD_DIR := build
BIN_DIR := build/bin
INC_DIR := $(PWD)
DEPS_DIR := ../third_party
TESTS_DIR := tests


# COMPILER SETTINGS
BUILD_TYPE := debug
# BUILD_TYPE := release
CC := gcc
# CC := tcc

STB_CFLAGS:=-I$(DEPS_DIR)/src/stb
SDL2_CFLAGS:=$(shell sdl2-config --cflags)
TIS_CFLAGS:=-DGST_DISABLE_DEPRECATED
TIS_CFLAGS+=$(shell pkg-config gstreamer-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gstreamer-video-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gobject-introspection-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config tcam --cflags)
CFLAGS := \
	-I$(INC_DIR) \
	-I$(DEPS_DIR)/include \
	$(SDL2_CFLAGS) \
	$(STB_CFLAGS) \
	$(TIS_CFLAGS) \
	-fopenmp

ifeq ($(BUILD_TYPE), debug)
	CFLAGS += -g -Wall -Wpedantic
else
	CFLAGS += -O3 -march=native -mtune=native -Wall -Wpedantic -DNDEBUG
endif

# LIBRARIES
SDL2_LIBS := `sdl2-config --libs` -lSDL2_image
GLEW_LIBS := -lGLEW
OPENGL_LIBS := $(SDL2_LIBS) $(GLEW_LIBS) -lGL
BLAS_LIBS := -lblas -llapack -llapacke
OPENSSL_LIBS := -lssl -lcrypto
CERES_DEPS := -lgflags -lglog \
							-llapack -lcamd -lamd -lccolamd -lcolamd -lcholmod \
							-lcxsparse
CERES_LIBS := -lceres $(CERES_DEPS)

TIS_LIBS:=-L$(DEP_DIR)/lib
TIS_LIBS+=$(shell pkg-config gstreamer-1.0 --libs)
TIS_LIBS+=$(shell pkg-config gstreamer-video-1.0 --libs)
TIS_LIBS+=$(shell pkg-config gobject-introspection-1.0 --libs)
TIS_LIBS+=$(shell pkg-config tcam --libs)
APRILTAG_LIBS := -L$(DEPS_DIR)/lib -lapriltag


# DEPENDENCIES
DEPS= \
  -L$(BLD_DIR) -lproto \
	$(APRILTAG_LIBS) \
	$(CERES_LIBS) \
	$(OPENGL_LIBS) \
	$(BLAS_LIBS) \
	$(OPENSSL_LIBS) \
	$(TIS_LIBS) \
	-lstdc++ \
	-lpthread \
  -lm


# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs


# TARGETS
LIBPROTO := $(BLD_DIR)/libproto.a
TESTS := \
	test_proto \
	test_aprilgrid \
	test_tis \
	test_sbgc \
	test_ubx

.PHONY: benchmarks build scripts shaders test_data viz


help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

$(BLD_DIR)/%.o: %.c %.h Makefile
	@echo "CC [$<]"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BLD_DIR)/ceres_bridge.o: ceres_bridge.cpp Makefile
	@echo "CXX [ceres_bridge.c]"
	@g++ -Wall -O3 -c ceres_bridge.cpp -o $(BLD_DIR)/ceres_bridge.o

$(BLD_DIR)/libproto.a: $(BLD_DIR)/proto.o $(BLD_DIR)/ceres_bridge.o
	@echo "AR [libproto.a]"
	@$(AR) $(ARFLAGS) \
		$(BLD_DIR)/libproto.a \
		$(BLD_DIR)/ceres_bridge.o \
		$(BLD_DIR)/proto.o \
		> /dev/null 2>&1

setup:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)
	@cp -r shaders $(BIN_DIR)
	@cp -r test_data $(BIN_DIR)

clean:  ## Clean build directory
	@rm -rf $(BLD_DIR)
	@rm -rf $(BIN_DIR)

libproto: setup $(BLD_DIR)/libproto.a               ## Compile libproto

test_proto: libproto    ## Compile test_proto
	@echo "CC [$@]"
	$(CC) $(CFLAGS) test_proto.c -o $(BIN_DIR)/test_proto $(DEPS)

test_aprilgrid:  ## Compile test_aprilgrid
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_aprilgrid.c -o $(BIN_DIR)/test_aprilgrid $(DEPS)

test_tis:  ## Compile test_tis
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_tis.c -o $(BIN_DIR)/test_tis $(DEPS)

test_sbgc: ## Compile test_sbgc
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_sbgc.c -o $(BIN_DIR)/test_sbgc $(DEPS)

test_ubx: ## Compile test_ubx
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_ubx.c -o $(BIN_DIR)/test_ubx $(DEPS)

tests: test_proto test_aprilgrid test_tis test_sbgc test_ubx
