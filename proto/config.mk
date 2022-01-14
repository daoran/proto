BLD_DIR := $(PWD)/build
BIN_DIR := $(PWD)/build/bin
INC_DIR := $(PWD)
DEPS_DIR := $(PWD)/../deps
TESTS_DIR := $(PWD)/tests

# COMPILER SETTINGS
# CC := tcc
# CC := gcc
CC := clang
CFLAGS := \
	-g \
	-Wall \
  -fsanitize=address \
	-Wpedantic \
	-I$(INC_DIR) \
	-I$(DEPS_DIR)/include \
	`sdl2-config --cflags`

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

# COMPILER COMMANDS
COMPILE_OBJ = \
	@echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_STATIC_LIB = \
	@echo "AR [libproto.a]"; \
	$(AR) $(ARFLAGS) $@ $^ > /dev/null 2>&1

COMPILE_TEST_OBJ = \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_TEST = \
	@echo "TEST [$(shell basename $@)]"; \
	$(CC) $(CFLAGS) $< -o $@ $(DEPS)
