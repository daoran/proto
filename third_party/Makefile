MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
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

build_pkgs: build_apriltag build_stb build_dbow2

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

build_tiscamera:
	@cd "$(SRC_PATH)/tiscamera" \
	  && ./scripts/dependency-manager install -y \
	  && mkdir -p build \
	  && cd build || return \
	  && cmake \
		  -DCMAKE_BUILD_TYPE=Release \
			-DTCAM_BUILD_ARAVIS=ON .. \
		&& make && sudo make install

build_arducam:
	@curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
	@sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
	@sudo apt update
	@sudo apt install arducam-config-parser-dev arducam-usb-sdk-dev
	@curl https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/releases/download/install_drivers/configure_udev_rules.sh -L -o /tmp/configure_udev_rules.sh
	@chmod +x /tmp/configure_udev_rules.sh
	@bash /tmp/configure_udev_rules.sh

build_stb:
	@cd "$(PREFIX)/include" \
    && ln -svf $(PWD)/src/stb/stb_c_lexer.h . \
    && ln -svf $(PWD)/src/stb/stb_connected_components.h . \
    && ln -svf $(PWD)/src/stb/stb_divide.h . \
    && ln -svf $(PWD)/src/stb/stb_ds.h . \
    && ln -svf $(PWD)/src/stb/stb_dxt.h . \
    && ln -svf $(PWD)/src/stb/stb_easy_font.h . \
    && ln -svf $(PWD)/src/stb/stb_herringbone_wang_tile.h . \
    && ln -svf $(PWD)/src/stb/stb_hexwave.h . \
    && ln -svf $(PWD)/src/stb/stb_image.h . \
    && ln -svf $(PWD)/src/stb/stb_image_resize.h . \
    && ln -svf $(PWD)/src/stb/stb_image_write.h . \
    && ln -svf $(PWD)/src/stb/stb_include.h . \
    && ln -svf $(PWD)/src/stb/stb_leakcheck.h . \
    && ln -svf $(PWD)/src/stb/stb_perlin.h . \
    && ln -svf $(PWD)/src/stb/stb_rect_pack.h . \
    && ln -svf $(PWD)/src/stb/stb_sprintf.h . \
    && ln -svf $(PWD)/src/stb/stb_textedit.h . \
    && ln -svf $(PWD)/src/stb/stb_tilemap_editor.h . \
    && ln -svf $(PWD)/src/stb/stb_truetype.h . \
    && ln -svf $(PWD)/src/stb/stb_voxel_render.h

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
