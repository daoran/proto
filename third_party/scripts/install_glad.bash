#!/bin/bash
set -e  # halt on first error
source "config.bash"

GLAD_DIR=$PREFIX/src/glad
SRC_DIR=$GLAD_DIR/src
INC_DIR=$GLAD_DIR/include
BLD_DIR=$GLAD_DIR/build

# Create directories
mkdir -p "$GLAD_DIR"
mkdir -p "$BLD_DIR"

# Build glad
cd "$PREFIX"/src
apt_install python3-pip python3-setuptools
sudo -H pip3 install wheel glad
glad --quiet --generator c --out glad
g++ -I"$INC_DIR" -fPIC -c "$SRC_DIR/glad.c" -o "$BLD_DIR/glad.o";
ar rs "$BLD_DIR/libglad.a" "$BLD_DIR/glad.o";

# Install glad
cp -r "$INC_DIR/glad" "$PREFIX/include";
cp -r "$INC_DIR/KHR" "$PREFIX/include";
cp "$BLD_DIR/libglad.a" "$PREFIX/lib/libglad.a";
