# Use the base `dustynv/ros` image with the desired L4T version
FROM dustynv/ros:noetic-ros-base-l4t-r32.7.1

# Install Python 3.7 and pip for Python 3.7
RUN apt-get update && apt-get install -y \
    python3.7 \
    python3.7-dev \
    python3.7-distutils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.7 as the default python version
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1

RUN python3 -m pip install --upgrade pip

# Install pip for Python 3.7
RUN ln -s /usr/bin/python3.7 /usr/local/bin/python3
