include ../config.mk

SRCS = $(wildcard *.c)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.c=.o))
LIB = $(BLD_DIR)/libzero.a

default: dirs shaders $(LIB)
.PHONY: tests dirs shaders

tests:
	@make -C tests

dirs:
	@mkdir -p $(BLD_DIR)

shaders:
	@cp -r shaders $(BIN_DIR)

$(BLD_DIR)/%.o: %.c %.h
	$(COMPILE_OBJ)

$(BLD_DIR)/libzero.a: $(OBJS)
	$(MAKE_STATIC_LIB)
