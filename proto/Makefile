include config.mk

default: $(BLD_DIR) $(BIN_DIR) lib tests done
.PHONY: deps format_code lib tests done

deps:
	@make -s -C deps -j8

clean:
	@rm -rf $(BLD_DIR)

$(BLD_DIR):
	@mkdir -p $(BLD_DIR)

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

format_code:
	@bash scripts/format_code.bash

lib:
	@make -s -C lib -j8

benchmarks:
	@make -s -C lib/benchmarks

tests:
	@make -s -C lib/tests -j8

done:
	@echo "Done! :)"
