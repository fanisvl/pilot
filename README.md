# autopilot

Before starting, run this command to make GUIs work inside a docker container:  
```xhost +local:docker```

1. Clone the repo

3. Download (https://github.com/AUEB-CS-Autonomous-Racing/autopilot/pkgs/container/autopilot%2Fautopilot) or build image
  ```docker build -t pilot .```

4. Run container  - This will also mount the repo you cloned to the docker container  
   Execute these commands inside **/autopilot**  
  No GPU
  ```
  docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/video0:/dev/video0 \
    -v .:/workspace/autopilot \
    pilot \
    bash
  ```

With GPU support - **not tested**
  ```
  docker run -it --net=host --gpus all \
      --env="NVIDIA_DRIVER_CAPABILITIES=all" \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --device=/dev/video0:/dev/video0 \
      -v .:/workspace/autopilot \
      pilot \
      bash
  ```

5. Build ROS autopilot packages **inside workspace/autopilot**
   ```colcon build```
   ```source install/setup.bash```

# Simulator Setup
Inside the same docker container and /workspace:  

1. Create sim directory
 ```
 mkdir sim && cd sim
 ```

2. Clone eufs_sim repository to /sim
```
git clone https://gitlab.com/eufs/eufs_sim.git
```
3. Clone eufs_msgs repository to /sim
```
git clone https://gitlab.com/eufs/eufs_msgs.git
```
4. Set EUFS_MASTER environment variable to the path of the directory of eufs_sim and eufs_msgs
```
echo 'export EUFS_MASTER=/workspace/sim' >> ~/.bashrc
```
5. call bash script to update environment variable above
```
source ~/.bashrc
```

6. Install rosdep
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update --rosdistro foxy
rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
```
7. inside /workspace/sim build the simulator (may need to run second time if it fails)
```
colcon build
```

8. Launch the simulator inside /workspace/sim
```
. install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

Common Erros
1. Visual Studio Code when trying to write to mounted file:
  Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
  Fix:
  ```sudo chown -R $USER .```

