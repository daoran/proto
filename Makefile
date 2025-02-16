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

setup:
	@mkdir -p $(BLD_DIR)
	@cp -r src/fonts $(BLD_DIR)
	@cp -r src/test_data $(BLD_DIR)

clean:  ## Clean
	@rm -rf $(BLD_DIR)

$(BLD_DIR)/test_%: src/test_%.c libxyz
	@echo "TEST [$(notdir $@)]"
	@$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS) -lxyz

$(BLD_DIR)/%.o: src/%.c src/%.h Makefile
	@echo "CC [$(notdir $<)]"
	$(CC) $(CFLAGS) -c $< -o $@

$(BLD_DIR)/xyz_ceres.o: src/xyz_ceres.cpp Makefile
	@echo "CXX [$(notdir $<)]"
	g++ -Wall -O3 \
		-c $< \
		-o $(BLD_DIR)/$(basename $(notdir $<)).o \
		-I/usr/include/eigen3

$(BLD_DIR)/libxyz.a: $(LIBXYZ_OBJS)
	@echo "AR [libxyz.a]"
	$(AR) $(ARFLAGS) \
		$(BLD_DIR)/libxyz.a \
		$(LIBXYZ_OBJS) \
		> /dev/null 2>&1

libxyz: setup $(BLD_DIR)/libxyz.a  ## Build libxyz

install: ## Install libxyz
	mkdir -p $(PREFIX)
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	ln -sf $(CUR_DIR)/xyz.py $(PYTHON3_PATH)/xyz.py
	ln -sf $(CUR_DIR)/build/libxyz.a $(PREFIX)/lib/libxyz.a
	ln -sf $(CUR_DIR)/*.h $(PREFIX)/include/*.h
	ln -sf $(CUR_DIR)/xyz.h            $(PREFIX)/include/xyz.h
	ln -sf $(CUR_DIR)/xyz_aprilgrid.h  $(PREFIX)/include/xyz_aprilgrid.h
	ln -sf $(CUR_DIR)/xyz_calib.h      $(PREFIX)/include/xyz_calib.h
	ln -sf $(CUR_DIR)/xyz_ceres.h      $(PREFIX)/include/xyz_ceres.h
	ln -sf $(CUR_DIR)/xyz_control.h    $(PREFIX)/include/xyz_control.h
	ln -sf $(CUR_DIR)/xyz_cv.h         $(PREFIX)/include/xyz_cv.h
	ln -sf $(CUR_DIR)/xyz_ds.h         $(PREFIX)/include/xyz_ds.h
	ln -sf $(CUR_DIR)/xyz_euroc.h      $(PREFIX)/include/xyz_euroc.h
	ln -sf $(CUR_DIR)/xyz_gimbal.h     $(PREFIX)/include/xyz_gimbal.h
	ln -sf $(CUR_DIR)/xyz_gnuplot.h    $(PREFIX)/include/xyz_gnuplot.h
	ln -sf $(CUR_DIR)/xyz_gui.h        $(PREFIX)/include/xyz_gui.h
	ln -sf $(CUR_DIR)/xyz_http.h       $(PREFIX)/include/xyz_http.h
	ln -sf $(CUR_DIR)/xyz_mav.h        $(PREFIX)/include/xyz_mav.h
	ln -sf $(CUR_DIR)/xyz_se.h         $(PREFIX)/include/xyz_se.h
	ln -sf $(CUR_DIR)/xyz_sim.h        $(PREFIX)/include/xyz_sim.h
	ln -sf $(CUR_DIR)/xyz_timeline.h   $(PREFIX)/include/xyz_timeline.h

uninstall: ## Uninstall libxyz
	rm $(PYTHON3_PATH)/xyz.py
	rm $(PREFIX)/lib/libxyz.a
	rm $(PREFIX)/include/xyz.h
	rm $(PREFIX)/include/xyz_aprilgrid.h
	rm $(PREFIX)/include/xyz_calib.h
	rm $(PREFIX)/include/xyz_ceres.h
	rm $(PREFIX)/include/xyz_control.h
	rm $(PREFIX)/include/xyz_cv.h
	rm $(PREFIX)/include/xyz_ds.h
	rm $(PREFIX)/include/xyz_euroc.h
	rm $(PREFIX)/include/xyz_gimbal.h
	rm $(PREFIX)/include/xyz_gnuplot.h
	rm $(PREFIX)/include/xyz_gui.h
	rm $(PREFIX)/include/xyz_http.h
	rm $(PREFIX)/include/xyz_mav.h
	rm $(PREFIX)/include/xyz_se.h
	rm $(PREFIX)/include/xyz_sim.h
	rm $(PREFIX)/include/xyz_timeline.h

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

tests: $(TESTS) ## Build tests

run_tests: tests ## Run tests
	@cd ./build && $(foreach TEST, $(TESTS), ./$(notdir ${TEST});)

ci: ## Run CI tests
	@make tests CI_MODE=1 --no-print-directory
	@cd ./build && $(foreach TEST, $(TESTS), ./$(notdir ${TEST});)

all: libxyz tests

cppcheck: ## Run cppcheck on xyz.c
	@cppcheck src/xyz.c src/xyz.h
