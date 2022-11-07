# PATHS
BLD_DIR := build
BIN_DIR := build/bin
INC_DIR := $(PWD)
DEPS_DIR := ../third_party
TESTS_DIR := tests


# COMPILER SETTINGS
BUILD_TYPE := debug
CC := gcc
# CC := tcc

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
	$(TIS_CFLAGS)

ifeq ($(BUILD_TYPE), debug)
	CFLAGS += -g -Wall -Wpedantic
else
	CFLAGS += -O3 -DNDEBUG
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
CERES_LIBS := -L$(DEPS_DIR)/lib -lceres $(CERES_DEPS)

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

.PHONY: dirs benchmarks scripts shaders test_data viz

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

default: dirs $(LIBPROTO) shaders test_data

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)

clean:  ## Clean build directory
	@rm -rf $(BLD_DIR)
	@rm -rf $(BIN_DIR)

shaders: dirs
	@cp -r shaders $(BIN_DIR)

test_data: dirs
	@cp -r test_data $(BIN_DIR)

libproto: dirs  ## Compile libproto
	@echo "Building [libproto.a]"
	@python3 scripts/make_cmodule.py --infile proto.h
	@echo "CC [proto.c]"
	@$(CC) $(CFLAGS) -c proto.c -o $(BLD_DIR)/proto.o
	@echo "AR [libproto.a]"
	@$(AR) $(ARFLAGS) $(BLD_DIR)/libproto.a $(BLD_DIR)/proto.o > /dev/null 2>&1
	@rm proto.c

test_proto: dirs  ## Compile libproto unittests
	@echo "CC [test_proto.c]"
	@python3 scripts/make_cmodule.py -in proto.h -test true
	@$(CC) $(CFLAGS) test_proto.c -o $(BIN_DIR)/test_proto $(DEPS)
	@rm test_proto.c

test_aprilgrid: dirs  ## Compile test_aprilgrid
	@echo "CC [test_aprilgrid.c]"
	@python3 scripts/make_cmodule.py -in aprilgrid.h -test true
	@$(CC) $(CFLAGS) test_aprilgrid.c -o $(BIN_DIR)/test_aprilgrid $(DEPS)
	@rm test_aprilgrid.c

test_tis: dirs  ## Compile test_tis
	@echo "CC [test_tis.c]"
	@python3 scripts/make_cmodule.py -in tis.h -test true
	@$(CC) $(CFLAGS) test_tis.c -o $(BIN_DIR)/test_tis $(DEPS)
	@rm test_tis.c

test_sbgc: dirs  ## Compile test_sbgc
	@echo "CC [test_sbgc.c]"
	@python3 scripts/make_cmodule.py -in sbgc.h -test true
	@$(CC) $(CFLAGS) test_sbgc.c -o $(BIN_DIR)/test_sbgc $(DEPS)
	@rm test_sbgc.c

test_ubx: dirs  ## Compile test_ubx
	@echo "CC [test_ubx.c]"
	@python3 scripts/make_cmodule.py -in ubx.h -test true
	@$(CC) $(CFLAGS) test_ubx.c -o $(BIN_DIR)/test_ubx $(DEPS)
	@rm test_ubx.c

tests: $(TESTS)
