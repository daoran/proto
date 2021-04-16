BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEPS_DIR=$(PWD)/../deps
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=tcc
# CC=gcc
CFLAGS=-g -Wall -I$(INC_DIR) -I$(DEPS_DIR)/include -ggdb

GLFW3_LIBS=-L$(DEPS_DIR)/lib -lglfw3 -lrt -lm -ldl
GLEW_LIBS=-lGLEW
OPENGL_LIBS=$(GLFW3_LIBS) $(GLEW_LIBS) -lGL -L/usr/X11R6/lib -lX11
BLAS_LIBS=-lblas -llapack

LIBS=-L$(BLD_DIR) \
	-lzero \
	$(OPENGL_LIBS) \
	$(BLAS_LIBS) \
	-lpthread

# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs

# COMPILER COMMANDS
COMPILE_OBJ = \
	@echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_STATIC_LIB = \
	@echo "AR [libzero.a]"; \
	$(AR) $(ARFLAGS) $@ $^

COMPILE_TEST_OBJ = \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_TEST = \
	echo "TEST [$(shell basename $@)]"; \
	$(CC) $(CFLAGS) \
		$(subst $(BIN_DIR), $(BLD_DIR), $@.o) \
		-o $@ \
		$(LIBS)
