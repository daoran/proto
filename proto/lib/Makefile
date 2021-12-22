include ../config.mk

LIB := $(BLD_DIR)/libproto.a
TEST := $(BIN_DIR)/test_proto

default: dirs $(LIB) shaders test_data $(TEST)
.PHONY: dirs shaders test_data

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)

shaders: dirs
	@cp -r shaders $(BIN_DIR)

test_data: dirs
	@cp -r test_data $(BIN_DIR)

$(BLD_DIR)/%.o: %.c %.h
	$(COMPILE_OBJ)

$(LIB): $(BLD_DIR)/gui.o $(BLD_DIR)/proto.o
	$(MAKE_STATIC_LIB)

$(BIN_DIR)/test_%: test_%.c $(LIB)
	$(MAKE_TEST)
