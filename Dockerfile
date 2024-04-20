# TODO: NVIDIA Image for Jetson?

# Base image for development
FROM ubuntu:20.04

# Set DEBIAN_FRONTEND to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages and enable repositories
RUN apt-get update && \
    apt-get install -y \
    software-properties-common \
    curl && \
    rm -rf /var/lib/apt/lists/* && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get upgrade -y

# Add ROS 2 GPG key and apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update

# Install ROS 2 desktop packages
RUN apt-get install -y ros-foxy-desktop python3-argcomplete python3-pip && \
    apt-get install -y ros-dev-tools

# Source ROS 2 setup script
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Set the working directory in the container
WORKDIR /workspace

# Install requirements.txt
COPY requirements.txt /workspace/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Set up entry point or command as needed
CMD ["/bin/bash"]