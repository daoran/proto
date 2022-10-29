# PATHS
BLD_DIR := build
BIN_DIR := build/bin
INC_DIR := $(PWD)
DEPS_DIR := third_party
TESTS_DIR := tests


# COMPILER SETTINGS
BUILD_TYPE := debug
CC := gcc

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

TIS_LIBS:=$(shell pkg-config gstreamer-1.0 --libs)
TIS_LIBS+=$(shell pkg-config gstreamer-video-1.0 --libs)
TIS_LIBS+=$(shell pkg-config gobject-introspection-1.0 --libs)
TIS_LIBS+=$(shell pkg-config tcam --libs)
APRILTAG_LIBS := -lapriltag


# DEPENDENCIES
DEPS=-L$(BLD_DIR) \
  -lproto \
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
	$(BIN_DIR)/test_proto \
	$(BIN_DIR)/test_aprilgrid \
	$(BIN_DIR)/test_tis \
	$(BIN_DIR)/test_sbgc \
	$(BIN_DIR)/test_ubx

default: dirs $(LIBPROTO) shaders test_data
.PHONY: dirs shaders test_data

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)

clean:
	@rm -rf $(BLD_DIR)
	@rm -rf $(BIN_DIR)

shaders: dirs
	@cp -r shaders $(BIN_DIR)

test_data: dirs
	@cp -r test_data $(BIN_DIR)

$(BLD_DIR)/%.o: %.c %.h
	@echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@

$(LIBPROTO): $(BLD_DIR)/proto.o
	@echo "AR [libproto.a]"; \
	$(AR) $(ARFLAGS) $@ $^ > /dev/null 2>&1

$(BIN_DIR)/test_%: test_%.c munit.h $(LIBPROTO)
	@echo "CC [$(shell basename $@)]"; \
	$(CC) $(CFLAGS) $< -o $@ $(DEPS)

tests: $(TESTS)
