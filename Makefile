include config.mk

.PHONY: third_party docs build install tests ci run_test_xyz clean
.PHONY: libxyz benchmarks build scripts shaders test_data viz

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
	@pip3 install sphinx
	@pip3 install sphinx-autobuild
	@sleep 3 && xdg-open http://127.0.0.1:8000 &
	@rm -rf docs/build
	@sphinx-autobuild docs/source docs/build/html

$(BLD_DIR)/%.o: src/%.c src/%.h Makefile
	@echo "CC [$<]"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BLD_DIR)/ceres_bridge.o: src/ceres_bridge.cpp Makefile
	@echo "CXX [ceres_bridge.c]"
	@g++ -Wall -O3 -c src/ceres_bridge.cpp -o $(BLD_DIR)/ceres_bridge.o -I/usr/include/eigen3

$(BLD_DIR)/libxyz.a: $(LIBXYZ_OBJS)
	@echo "AR [libxyz.a]"
	@$(AR) $(ARFLAGS) \
		$(BLD_DIR)/libxyz.a \
		$(LIBXYZ_OBJS) \
		> /dev/null 2>&1

setup:
	@mkdir -p $(BLD_DIR)
	@cp -r src/shaders $(BLD_DIR)
	@cp -r src/test_data $(BLD_DIR)

clean:  ## Clean
	@rm -rf $(BLD_DIR)

libxyz: setup $(BLD_DIR)/libxyz.a  ## Build libxyz

install: ## Install libxyz
	mkdir -p $(PREFIX)
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	ln -sf $(CUR_DIR)/build/libxyz.a $(PREFIX)/lib/libxyz.a
	ln -sf $(CUR_DIR)/aprilgrid.h $(PREFIX)/include/aprilgrid.h
	ln -sf $(CUR_DIR)/ceres_bridge.h $(PREFIX)/include/ceres_bridge.h
	ln -sf $(CUR_DIR)/euroc.h $(PREFIX)/include/euroc.h
	ln -sf $(CUR_DIR)/gui.h $(PREFIX)/include/gui.h
	ln -sf $(CUR_DIR)/http.h $(PREFIX)/include/http.h
	ln -sf $(CUR_DIR)/xyz.h $(PREFIX)/include/xyz.h
	ln -sf $(CUR_DIR)/sbgc.h $(PREFIX)/include/sbgc.h
	ln -sf $(CUR_DIR)/stb_image.h $(PREFIX)/include/stb_image.h
	ln -sf $(CUR_DIR)/xyz.py $(PYTHON3_PATH)/xyz.py

uninstall: ## Uninstall libxyz
	rm $(PREFIX)/lib/libxyz.a
	rm $(PREFIX)/include/aprilgrid.h
	rm $(PREFIX)/include/euroc.h
	rm $(PREFIX)/include/gui.h
	rm $(PREFIX)/include/http.h
	rm $(PREFIX)/include/xyz.h
	rm $(PREFIX)/include/sbgc.h
	rm $(PREFIX)/include/stb_image.h
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

test_gui:  ## Run test_gui
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

tests: test_aprilgrid test_gui test_ubx test_xyz  ## Run tests

ci:  ## Run CI tests
	@$(CC) $(CFLAGS) -DMU_REDIRECT_STREAMS=1 src/test_xyz.c -o $(BLD_DIR)/test_xyz $(LDFLAGS)
	@cd build && ./test_xyz
