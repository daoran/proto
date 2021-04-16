include ../../config.mk

SRCS = $(wildcard *.c)
TESTS = $(addprefix $(BIN_DIR)/, $(SRCS:.c=))

all: $(TESTS) test_data scripts
.PHONY: test_data scripts

$(BLD_DIR)/%.o: %.c $(BLD_DIR)/libzero.a
	$(COMPILE_TEST_OBJ)

$(BIN_DIR)/test_%: $(BLD_DIR)/test_%.o
	$(MAKE_TEST)

test_data:
	@cp -r test_data $(BIN_DIR)

scripts:
	@cp -r scripts $(BIN_DIR)
