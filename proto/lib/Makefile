include config.mk

LIB := $(BLD_DIR)/libproto.a
TEST := $(BIN_DIR)/test_proto

default: dirs $(LIB) $(TEST)
.PHONY: dirs shaders run_tests clean tests_data

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)

shaders:
	@mkdir -p $(BIN_DIR)/shaders
	@cp -r shaders $(BIN_DIR)

run_tests: dirs shaders $(LIB) $(TEST)
	./build/bin/test_proto

clean:
	@rm -rf build

$(BLD_DIR)/%.o: %.c %.h
	$(COMPILE_OBJ)

$(LIB): $(BLD_DIR)/gui.o $(BLD_DIR)/proto.o
	$(MAKE_STATIC_LIB)

$(BIN_DIR)/test_%: test_%.c
	$(MAKE_TEST)
