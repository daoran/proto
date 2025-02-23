ROS_VERSION=humble
ARCH=$(dpkg --print-architecture)
ROS2_GPG=/usr/share/keyrings/ros-archive-keyring.gpg
ROS2_PKGS_URL=http://packages.ros.org/ros2/ubuntu
UBUNTU_CODENAME=$(. /etc/os-release && echo $UBUNTU_CODENAME)

# Install deps
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl

# Install ROS2 GPG key
sudo curl \
  -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo \
  "deb [arch=$ARCH signed-by=$ROS2_GPG] $ROS2_PKGS_URL $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt-get update -y -qq
sudo apt install -y -q \
  ros-$ROS_VERSION-desktop \
  python3-argcomplete \
  ros-dev-tools
