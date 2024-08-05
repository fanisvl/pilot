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
  -e DISPLAY=$DISPLAY \
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

# Simulator Setup
Inside the same docker container and /workspace:  

1. Create sim directory, clone eufs_sim and eufs_msgs
 ```
 mkdir sim && cd sim
 git clone https://gitlab.com/eufs/eufs_sim.git
 git clone https://gitlab.com/eufs/eufs_msgs.git
 ```

2. Set EUFS_MASTER environment variable to the path of the directory of eufs_sim and eufs_msgs
```
echo 'export EUFS_MASTER=/workspace/sim' >> ~/.bashrc
source ~/.bashrc
```

3. Install dependencies with rosdep
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update --rosdistro foxy
rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
```

4. inside /workspace/sim build the simulator (may need to run second time if it fails)
```
colcon build
```

5. Launch the simulator inside /workspace/sim
```
. install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

# Common Erros

#### 1. Authorization required, but no authorization protocol specified. This error usually occurs when trying to launch a GUI
```
Authorization required, but no authorization protocol specified
could not connect to display :0
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix  this problem.
Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
```
**Fix:** Run ```xhost +local:docker``` from a local terminal.

#### 2. model.pt not found eg.
```
ros2 run vision stereo_cam_sub
FileNotFoundError: [Errno 2] No such file or directory: 'model/yolov8s700.pt'
```
**Fix:** The ros2 run command should be ran from the autopilot/ directory.

#### 3. ROS2 not found when launching the simulator
```
bash: ros2 command not found
```
**Fix:** ```source /opt/ros/$ROS_DISTRO/setup.bash```

#### 4. Visual Studio Code when trying to write to mounted file:
```
Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
```
**Fix:** ```sudo chown -R $USER .```
