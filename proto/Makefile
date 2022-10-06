# PATHS
BLD_DIR := $(PWD)/build
BIN_DIR := $(PWD)/build/bin
INC_DIR := $(PWD)
DEPS_DIR := $(PWD)/deps
TESTS_DIR := $(PWD)/tests


# COMPILER SETTINGS
CC := gcc

SDL2_CFLAGS:=$(shell sdl2-config --cflags)
TIS_CFLAGS:=-DGST_DISABLE_DEPRECATED
TIS_CFLAGS+=$(shell pkg-config gstreamer-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gstreamer-video-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gobject-introspection-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config tcam --cflags)
CFLAGS := \
	-g \
	-Wall \
	-Wpedantic \
	-I$(INC_DIR) \
	-I$(DEPS_DIR)/include \
	$(SDL2_CFLAGS) \
	$(TIS_CFLAGS)



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


# DEPENDENCIES
DEPS=-L$(BLD_DIR) \
  -lproto \
	$(CERES_LIBS) \
	$(OPENGL_LIBS) \
	$(BLAS_LIBS) \
	$(OPENSSL_LIBS) \
	-lpthread \
  -lm


# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs


# TARGETS
LIB := $(BLD_DIR)/libproto.a
TEST_PROTO := $(BIN_DIR)/test_proto
TEST_SBGC := $(BIN_DIR)/test_sbgc
TEST_UBX := $(BIN_DIR)/test_ubx

default: dirs $(LIB) shaders test_data $(TEST_PROTO) $(TEST_SBGC) $(TEST_UBX)
.PHONY: dirs shaders test_data tests

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

$(LIB): $(BLD_DIR)/proto.o
	@echo "AR [libproto.a]"; \
	$(AR) $(ARFLAGS) $@ $^ > /dev/null 2>&1

$(BIN_DIR)/test_%: test_%.c munit.h $(LIB)
	echo "TEST [$(shell basename $@)]"; \
	$(CC) $(CFLAGS) $< -o $@ $(DEPS)

tiscam_driver: tiscam_driver.c
	$(CC) $(CFLAGS) $< -o $(BIN_DIR)/$@ $(TIS_LIBS) $(SDL2_LIBS)

tests: $(LIB) $(BIN_DIR)/test_proto
	$(BIN_DIR)/test_proto
