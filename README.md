# autopilot

Before starting, run this command to make GUIs work inside a docker container:  
```xhost +local:docker```

1. Clone the repo

3. Download (https://github.com/AUEB-CS-Autonomous-Racing/autopilot/pkgs/container/autopilot%2Fautopilot) or build image
  ```docker build -t pilot .```

4. Run container - This will also mount the repo you cloned to the docker container

Note: These commands should be ran inside /autopilot  
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

With GPU support - (not tested)
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

5. Build ROS packages
   ```colcon build```
   ```source install/setup.bash```
   
6. RUN EACH COMMAND ONLY ONCE STARTING FROM TOP TO BOTTOM ONE BY ONE

  6.1 Clone repository of eufs_sim to /workspace
```
git clone https://gitlab.com/eufs/eufs_sim.git
```
  6.2 Clone repository of eufs_msgs to /workspace
```
git clone https://gitlab.com/eufs/eufs_msgs.git
```
  6.3 set EUFS_MASTER environment variable to the path of the directory of eufs_sim and eufs_msgs
```
echo 'export EUFS_MASTER=/workspace' >> ~/.bashrc
```
  6.4 call bash script to update environment variable above
```
source ~/.bashrc
```

  6.5 Install rosdep
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update --rosdistro foxy
rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
```
  6.6 inside /workspace build the simulator
```
colcon build
```
  6.7 may need to run second time if it fails
# colcon build 

  6.8 Launch the simulator inside /workspace
```
. install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```
Common Erros
1. Visual Studio Code when trying to write to mounted file:
  Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
  Fix:
  ```sudo chown -R $USER .```

