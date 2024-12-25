MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src

default: dirs install
.PHONY: dirs install

setup:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share
	@rm -rf src

install: setup apt_base install_apt_pkgs build_pkgs

apt_base:
	@sudo apt-get install -y -qqq \
		dialog \
		apt-utils \
		git \
		mercurial \
		cmake \
		g++ \
		clang \
		tcc \
		libboost-dev \
		libyaml-dev \
		python3-pip \
		python3-setuptools
	@pip3 install --upgrade pip
	@pip3 install --user dataclasses

install_apt_pkgs:
	@sudo apt-get update -qqq
	@sudo apt-get install -y -qqq liblapacke liblapacke-dev
	@sudo apt-get install -y -qqq libsuitesparse-dev libsuitesparse-doc
	@sudo apt-get install -y -qqq libeigen3-dev libeigen3-doc
	@sudo apt-get install -y -qqq libceres-dev
	@sudo apt-get install -y -qqq libomp-dev
	@sudo apt-get install -y -qqq libopencv-*
	@sudo apt-get install -y -qqq libglew-dev
	@sudo apt-get install -y -qqq libsdl2-*
	@sudo apt-get install -y -qqq libassimp-dev libassimp-doc
	@sudo apt-get install -y -qqq libyaml-cpp-dev

build_pkgs: build_apriltag build_glfw build_stb build_dbow2

build_apriltag:
	@sudo apt-get install -y -qqq cmake libeigen3-dev libv4l-dev libopencv-*
	@cd "$(SRC_PATH)/apriltag" \
	  && mkdir -p build \
	  && cd build || return \
	  && cmake \
		  -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make \
		&& make install

build_khr:
	@cd "$(SRC_PATH)/KHR" \
		&& mkdir -p "$(PREFIX)/include/KHR" \
		&& ln -sf $(SRC_PATH)/KHR/khrplatform.h "$(PREFIX)/include/KHR"

build_glad: build_khr
	@cd "$(SRC_PATH)/glad" \
		&& mkdir -p "$(PREFIX)/include/glad" \
		&& ln -sf $(SRC_PATH)/glad/glad.h "$(PREFIX)/include/glad" \
		&& gcc -I$(PREFIX)/include -c glad.c -o glad.o \
		&& ar rcs libglad.a glad.o \
		&& ln -sf $(SRC_PATH)/glad/libglad.a "$(PREFIX)/lib/libglad.a"


build_glfw: build_glad
	@cd "$(SRC_PATH)/glfw" \
	  && mkdir -p build \
	  && cd build || return \
	  && cmake \
		  -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make \
		&& make install

build_stb:
	@cd "$(PREFIX)/include" \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_c_lexer.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_connected_components.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_divide.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_ds.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_dxt.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_easy_font.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_herringbone_wang_tile.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_hexwave.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image_resize.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_image_write.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_include.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_leakcheck.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_perlin.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_rect_pack.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_sprintf.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_textedit.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_tilemap_editor.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_truetype.h . \
		&& ln -svf $(MKFILE_DIR)/src/stb/stb_voxel_render.h

build_dbow2:
	@cd "$(SRC_PATH)/DBoW2" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
		  -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
		  .. \
		&& make \
		&& make install

build_fbow:
	@cd "$(SRC_PATH)/fbow" \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX="$(PREFIX)" \
			.. \
		&& make \
		&& make install
