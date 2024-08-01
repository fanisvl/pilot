FROM timongentzsch/l4t-ubuntu20-ros2-base:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install Ultralytics with compatible PyTorch whl (q-engineering)

# Install dependencies
RUN apt-get update && \
    apt-get install -y python3-pip libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev libopenblas-base libopenmpi-dev libomp-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install future && \
    pip3 install -U --user wheel mock pillow && \
    pip3 install testresources && \
    pip3 install setuptools==58.3.0 && \
    pip3 install Cython && \
    pip3 install gdown

# Install PyTorch
RUN gdown https://drive.google.com/uc?id=1e9FDGt2zGS5C5Pms7wzHYRb0HuupngK1 && \
    pip3 install torch-1.13.0a0+git7c98e70-cp38-cp38-linux_aarch64.whl && \
    rm torch-1.13.0a0+git7c98e70-cp38-cp38-linux_aarch64.whl

# Install TorchVision 0.14.0
RUN gdown https://drive.google.com/uc?id=19UbYsKHhKnyeJ12VPUwcSvoxJaX7jQZ2 && \
    pip3 install torchvision-0.14.0a0+5ce4506-cp38-cp38-linux_aarch64.whl && \
    rm torchvision-0.14.0a0+5ce4506-cp38-cp38-linux_aarch64.whl

# Manually install other dependencies required by ultralytics
RUN pip3 install \
    gitpython>=3.1.30 \
    matplotlib>=3.3 \
    numpy>=1.23.5 \
    pillow>=10.3.0 \
    psutil \
    PyYAML>=5.3.1 \
    thop>=0.1.1 \
    tqdm>=4.64.0 \
    py-cpuinfo \
    seaborn>=0.11.0 \
    ultralytics-thop>=2.0.0 \
    opencv-python==4.5.5.64 \
    requests>=2.23.0 \
    python-dateutil>=2.8.2 \
    pandas>=1.1.4 \
    cv_bridge 

RUN pip3 install --no-deps ultralytics

WORKDIR /
RUN echo "source workspace/autopilot/install/setup.bash" >> ~/.bashrc
# Entry point for the container
ENTRYPOINT ["/bin/bash", "-c", "exec bash"]

