include config.mk

default: $(BLD_DIR) $(BIN_DIR) zero tests done
.PHONY: format_code zero tests done

clean:
	@rm -rf $(BLD_DIR)

$(BLD_DIR):
	@mkdir -p $(BLD_DIR)

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

format_code:
	@bash scripts/format_code.bash

zero:
	@make -s -C zero -j8

benchmarks:
	@make -s -C zero/benchmarks

tests:
	@make -s -C zero/tests -j8

done:
	@echo "Done! :)"
