# Use NVIDIA's L4T base image for Jetson
FROM nvcr.io/nvidia/l4t-base:r32.4.3

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

# Add ROS Noetic GPG key and apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros-noetic.list > /dev/null && \
    apt-get update

# Install ROS Noetic desktop packages
RUN apt-get install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    apt-get install -y ros-dev-tools

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Source ROS Noetic setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Set the working directory in the container
WORKDIR /workspace

# Install requirements.txt
COPY requirements.txt /workspace/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Set up entry point or command as needed
CMD ["/bin/bash"]