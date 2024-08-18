# autopilot

Before starting, run this command to make GUIs work inside a docker container:  
```xhost +local:docker```

1. Clone the repo

2. Execute command - This pull the correct image, run a container  and mount the autopilot dir to the docker container  
   Execute this command inside **/autopilot**  
  No GPU
  ```
  docker run -it \
  --net=host \
  --ulimit nofile=1024:524288 \
  --env="DISPLAY=:0" \
  --env="QT_X11_NO_MITSHM=1" \
  -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/dri:/dev/dri \
  -v ~/repos/autopilot:/workspace/autopilot \
  fanisvl/ub20-py38-ros1noetic-pytorch:latest \
  bash
  ```
  Install CPU Pytorch once inside the container
  ```
    pip3 uninstall torch torchvision
    pip3 install torch torchvision
  ```


Jetson with GPU Support
  ```
  docker run -it \
    --net=host \
    --gpus all \
    --ulimit nofile=1024:524288 \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/video0:/dev/video1 \
    -v ~/workspace:/workspace \
    fanisvl/ub20-py38-ros1noetic-pytorch:latest \
    bash
  ```

3. Build ROS autopilot catkin workspace & packages - **inside workspace/autopilot**
   ```catkin_make```
   ```source devel/setup.bash```
   ```echo "source /workspace/autopilot/devel/setup.bash" >> ~/.bashrc" ```

# Common Erros

#### Authorization required, but no authorization protocol specified. This error usually occurs when trying to launch a GUI inside docker.
```
Authorization required, but no authorization protocol specified
could not connect to display :0
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix  this problem.
Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
```
**Fix:** Run ```xhost +local:docker``` from a local terminal.

#### bash: ros command not found
**Fix:** ```source /opt/ros/$ROS_DISTRO/setup.bash```

#### Visual Studio Code when trying to write to mounted file:
```
Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
```
**Fix:** ```sudo chown -R $USER .```

#### roscore eats all the RAM
``` ulimit -Sn 524288 && ulimit -Hn 524288 ```
