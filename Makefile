include config.mk
.PHONY: benchmarks build docs scripts src third_party

help:
	@echo "make targets:"
	@echo "----------------------------------------"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' Makefile \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "%-16s%s\n", $$1, $$2}'

third_party: ## Install dependencies
	@git submodule init
	@git submodule update
	@make -s -C third_party

docs: ## Build docs
	@livereload .

$(BLD_DIR)/%.o: src/%.c src/%.h Makefile
	@echo "CC [$<]"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BLD_DIR)/xyz_ceres_bridge.o: src/xyz_ceres_bridge.cpp Makefile
	@echo "CXX [xyz_ceres_bridge.c]"
	@g++ -Wall -O3 \
		-c src/xyz_ceres_bridge.cpp \
		-o $(BLD_DIR)/xyz_ceres_bridge.o \
		-I/usr/include/eigen3

$(BLD_DIR)/libxyz.a: $(LIBXYZ_OBJS)
	@echo "AR [libxyz.a]"
	@$(AR) $(ARFLAGS) \
		$(BLD_DIR)/libxyz.a \
		$(LIBXYZ_OBJS) \
		> /dev/null 2>&1

setup:
	@mkdir -p $(BLD_DIR)
	@cp -r src/fonts $(BLD_DIR)
	@cp -r src/test_data $(BLD_DIR)

clean:  ## Clean
	@rm -rf $(BLD_DIR)

libxyz: setup $(BLD_DIR)/libxyz.a  ## Build libxyz

install: ## Install libxyz
	mkdir -p $(PREFIX)
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	ln -sf $(CUR_DIR)/build/libxyz.a $(PREFIX)/lib/libxyz.a
	ln -sf $(CUR_DIR)/xyz.py $(PYTHON3_PATH)/xyz.py
	ln -sf $(CUR_DIR)/xyz.h $(PREFIX)/include/xyz.h
	ln -sf $(CUR_DIR)/xyz_aprilgrid.h $(PREFIX)/include/xyz_aprilgrid.h
	ln -sf $(CUR_DIR)/xyz_ceres_bridge.h $(PREFIX)/include/xyz_ceres_bridge.h
	ln -sf $(CUR_DIR)/xyz_euroc.h $(PREFIX)/include/xyz_euroc.h
	ln -sf $(CUR_DIR)/xyz_gui.h $(PREFIX)/include/xyz_gui.h
	ln -sf $(CUR_DIR)/xyz_http.h $(PREFIX)/include/xyz_http.h
	ln -sf $(CUR_DIR)/stb_image.h $(PREFIX)/include/stb_image.h

uninstall: ## Uninstall libxyz
	rm $(PREFIX)/lib/libxyz.a
	rm $(PREFIX)/include/xyz.h
	rm $(PREFIX)/include/xyz_aprilgrid.h
	rm $(PREFIX)/include/xyz_ceres_bridge.h
	rm $(PREFIX)/include/xyz_euroc.h
	rm $(PREFIX)/include/xyz_gui.h
	rm $(PREFIX)/include/xyz_http.h
	rm $(PREFIX)/include/xyz_sbgc.h
	rm $(PREFIX)/include/xyz_stb_image.h
	rm $(PYTHON3_PATH)/xyz.py

avs: $(BLD_DIR)/libxyz.a
	@g++ \
		-std=c++17 \
		-fsanitize=address -static-libasan \
		-fopenmp \
		-g \
		-I$(DEPS_DIR)/include \
		-I/usr/include/eigen3 \
		$(shell pkg-config opencv4 --cflags) \
		avs.cpp \
		-o $(BLD_DIR)/avs \
		$(LDFLAGS) \
		-lxyz \
		$(shell pkg-config opencv4 --libs)

test_xyz: libxyz  ## Run test_xyz
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_xyz.c -o $(BLD_DIR)/test_xyz $(LDFLAGS)
	@cd build && ./test_xyz --target $(TEST_TARGET)

test_aprilgrid:  ## Run test_aprilgrid
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_aprilgrid.c -o $(BLD_DIR)/test_aprilgrid $(LDFLAGS)
	@cd build && ./test_aprilgrid

test_euroc:  ## Run test_euroc
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_euroc.c -o $(BLD_DIR)/test_euroc $(LDFLAGS)
	@cd build && ./test_euroc

test_gui: ## Run test_gui
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_gui.c -o $(BLD_DIR)/test_gui $(LDFLAGS)

test_sbgc:  ## Run test_sbgc
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_sbgc.c -o $(BLD_DIR)/test_sbgc $(LDFLAGS)
	@cd build && ./test_sbgc

test_ubx:  ## Run test_ubx
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_ubx.c -o $(BLD_DIR)/test_ubx $(LDFLAGS)
	@cd build && ./test_ubx

test_http:  ## Run test_http
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) src/test_http.c -o $(BLD_DIR)/test_http $(LDFLAGS)
	@cd build && ./test_http

tests: test_aprilgrid test_gui test_xyz  ## Run tests

ci:  ## Run CI tests
	@$(CC) $(CFLAGS) -DMU_REDIRECT_STREAMS=1 src/test_xyz.c -o $(BLD_DIR)/test_xyz $(LDFLAGS)
	@cd build && ./test_xyz

cppcheck: ## Run cppcheck on xyz.c
	@cppcheck src/xyz.c src/xyz.h
