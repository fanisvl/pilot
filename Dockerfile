# Use Nvidia's CUDA + OpenGL base image with Ubuntu 20.04
FROM nvidia/cudagl:11.1.1-base-ubuntu20.04

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Install necessary packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
 && rm -rf /var/lib/apt/lists/*

# Enable Ubuntu Universe repository
RUN add-apt-repository universe

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 apt repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt repository caches
RUN apt-get update

# Upgrade existing packages
RUN apt-get upgrade -y

# Install ROS 2 desktop packages
RUN apt-get install -y ros-foxy-desktop python3-argcomplete python3-pip

# Source ROS 2 setup script
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# (Optional) Install ROS 2 base packages or development tools if needed
RUN apt-get install -y ros-dev-tools

# (ROS-Base instead of ros-desktop) RUN apt-get install -y ros-foxy-ros-base python3-argcomplete

# Set the working directory in the container
WORKDIR /workspace

# Set up entry point or command as needed
CMD ["/bin/bash"]
