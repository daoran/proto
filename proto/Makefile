# PATHS
BLD_DIR := build
INC_DIR := $(realpath $(PWD))
DEPS_DIR := $(realpath ../third_party)


# COMPILER SETTINGS
BUILD_TYPE := debug
# BUILD_TYPE := release
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
CERES_LDFLAGS := -lgflags -lglog -lceres

TIS_CFLAGS:=-DGST_DISABLE_DEPRECATED
TIS_CFLAGS+=$(shell pkg-config gstreamer-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gstreamer-video-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config gobject-introspection-1.0 --cflags)
TIS_CFLAGS+=$(shell pkg-config tcam --cflags)
TIS_LDFLAGS:=-L$(DEP_DIR)/lib
TIS_LDFLAGS+=$(shell pkg-config gstreamer-1.0 --libs)
TIS_LDFLAGS+=$(shell pkg-config gstreamer-video-1.0 --libs)
TIS_LDFLAGS+=$(shell pkg-config gobject-introspection-1.0 --libs)
TIS_LDFLAGS+=$(shell pkg-config tcam --libs)

APRILTAG_LDFLAGS := -L$(DEPS_DIR)/lib -lapriltag

YAML_LDFLAGS := -lyaml

PROTO_LDFLAGS := -L$(BLD_DIR) -lproto

# CFLAGS
CFLAGS := \
	-I$(INC_DIR) \
	-I$(DEPS_DIR)/include \
	$(SDL2_CFLAGS) \
	$(STB_CFLAGS) \
	$(TIS_CFLAGS)

ifeq ($(BUILD_TYPE), debug)
	CFLAGS += -g -Wall -Wpedantic -fsanitize=address -static-libasan -fopenmp
else
	CFLAGS += -O3 -march=native -mtune=native -Wall -Wpedantic -DNDEBUG -fopenmp
endif



# LDFLAGS
RPATH := -Wl,-rpath,$(DEPS_DIR)/lib
LDFLAGS= \
	$(RPATH) \
  $(PROTO_LDFLAGS) \
	$(CERES_LDFLAGS) \
	$(APRILTAG_LDFLAGS) \
	$(OPENGL_LDFLAGS) \
	$(SUITESPARSE_LDFLAGS) \
	$(BLAS_LDFLAGS) \
	$(OPENSSL_LDFLAGS) \
	$(TIS_LDFLAGS) \
	$(YAML_LDFLAGS) \
	-lstdc++ \
	-lpthread \
  -lm


# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs


# TARGETS
LIBPROTO := $(BLD_DIR)/libproto.a
LIBPROTO_OBJS := \
	$(BLD_DIR)/proto.o \
	$(BLD_DIR)/ceres_bridge.o \
	$(BLD_DIR)/aprilgrid.o \
	$(BLD_DIR)/euroc.o \
	$(BLD_DIR)/sbgc.o \
	$(BLD_DIR)/tis.o

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

$(BLD_DIR)/libproto.a: $(LIBPROTO_OBJS)
	@echo "AR [libproto.a]"
	@$(AR) $(ARFLAGS) \
		$(BLD_DIR)/libproto.a \
		$(LIBPROTO_OBJS) \
		> /dev/null 2>&1

setup:
	@mkdir -p $(BLD_DIR)
	@cp -r shaders $(BLD_DIR)
	@cp -r test_data $(BLD_DIR)

clean:  ## Clean build directory
	@rm -rf $(BLD_DIR)

libproto: setup $(BLD_DIR)/libproto.a  ## Compile libproto

avs: $(BLD_DIR)/libproto.a
	@g++ \
		-std=c++11 \
		-fsanitize=address -static-libasan \
		-fopenmp \
		-I$(DEPS_DIR)/include \
		$(shell pkg-config opencv4 --cflags) \
		avs.cpp \
		-o $(BLD_DIR)/avs \
		$(LDFLAGS) \
		-lproto \
		$(shell pkg-config opencv4 --libs)

test_proto: libproto  ## Compile test_proto
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_proto.c -o $(BLD_DIR)/test_proto $(LDFLAGS)

test_aprilgrid:  ## Compile test_aprilgrid
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_aprilgrid.c -o $(BLD_DIR)/test_aprilgrid $(LDFLAGS)

test_euroc:  ## Compile test_euroc
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_euroc.c -o $(BLD_DIR)/test_euroc $(LDFLAGS)

test_tis:  ## Compile test_tis
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_tis.c -o $(BLD_DIR)/test_tis $(LDFLAGS)

test_sbgc:  ## Compile test_sbgc
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_sbgc.c -o $(BLD_DIR)/test_sbgc $(LDFLAGS)

test_ubx:  ## Compile test_ubx
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_ubx.c -o $(BLD_DIR)/test_ubx $(LDFLAGS)

test_http:  ## Compile test_http
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) test_http.c -o $(BLD_DIR)/test_http $(LDFLAGS)

tests: test_proto test_aprilgrid test_tis test_sbgc test_ubx
