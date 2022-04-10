include config.mk

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
	$(COMPILE_OBJ)

$(LIB): $(BLD_DIR)/proto.o
	$(MAKE_STATIC_LIB)

$(BIN_DIR)/test_%: test_%.c munit.h $(LIB)
	$(MAKE_TEST)

tests: $(LIB) $(BIN_DIR)/test_proto
	$(BIN_DIR)/test_proto
