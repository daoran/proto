# Configurations
BUILD_TYPE="Release"
ROS_VERSION="noetic"

# $1 - Directory to check
check_dir() {
  if [ ! -d "$1" ]; then
    echo "DIR [$1] does not exist!";
    exit -1;
  fi
}

apt_update() {
  echo -n "[Updating APT package list]";
  if sudo apt-get update -qqq > log/update.log 2>&1 ; then
    echo -e "\033[0;32m OK! \033[0m"
  else
    echo -e "\033[0;31m FAILED!: \033[0m"
    cat log/update.log
    exit -1
  fi
}

apt_install() {
  sudo apt-get install -qqq -y "$@"
}

install() {
  echo -n "Installing $1 ..."
  if "$SCRIPTPATH/scripts/install_$1.bash" > log/install.log 2>&1; then
    echo -e "\033[0;32m OK! \033[0m"
  else
    echo -e "\033[0;31m FAILED! \033[0m"
    cat log/install.log
    exit -1
  fi
}

install_base() {
  apt_install dialog apt-utils git mercurial cmake g++ clang tcc
  apt_install python3-pip python3-setuptools

  pip3 install --upgrade pip
  pip3 install --user dataclasses
  pip3 install --user numpy
  pip3 install --user scipy
  pip3 install --user scikit-build
  pip3 install --user pandas
  pip3 install --user opencv-python
  pip3 install --user matplotlib
}

build_cmake_project() {
  # Go into repo
  cd "$SRC_PATH/$1" || return

  # Prepare for build
  mkdir -p build
  cd build || return
  cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX="$PREFIX"

  # Compile and install
  make -j2 && make install
}

# $1 - Git Repo URL
# $2 - Destination
clone_git_repo() {
  cd "$SRC_PATH" || exit
  if [ ! -d "$2" ]; then
    git clone "$1" "$2"
  fi
  cd - > /dev/null || exit
}

# $1 - Mercurial Repo URL
# $2 - Repo folder name
clone_hg_repo() {
  # Clone repo
  mkdir -p "$SRC_PATH"
  cd "$SRC_PATH" || return
  if [ ! -d "$2" ]; then
    hg clone "$1" "$2"
  fi
}

# "$1" - Repo URL
# "$2" - Repo folder name
download_zip_repo() {
  # Download repo
  mkdir -p "$SRC_PATH"
  cd "$SRC_PATH" || return
  if [ ! -f "$2".zip ]; then
    wget --no-check-certificate "$1" -O "$2".zip
  fi
  unzip -oqq "$2".zip
  cd "$2" || return
}
