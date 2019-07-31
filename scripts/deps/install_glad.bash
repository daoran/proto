#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

GLAD_DIR=$PREFIX/src/glad
SRC_DIR=$GLAD_DIR/src
INC_DIR=$GLAD_DIR/include
BLD_DIR=$GLAD_DIR/build

# Create directories
mkdir -p "$GLAD_DIR"
mkdir -p "$BLD_DIR"

# Build glad
cd "$GLAD_DIR"
pip install glad
glad --quiet --generator c --out glad
g++ -I"$INC_DIR" -c "$SRC_DIR/glad.c" -o "$BLD_DIR/glad.o";
ar rs "$BLD_DIR/libglad.a" "$BLD_DIR/glad.o";

# Install glad
cp -r "$INC_DIR" "$PREFIX/include";
cp "$BLD_DIR/libglad.a" "$PREFIX/lib/libglad.a";
