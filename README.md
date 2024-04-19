# autopilot

Before starting, run this command to make GUIs work inside a docker container:  
```xhost +local:docker```

1. Clone the repo

3. Inside the cloned directory, build the image using the Dockefile
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
    --device=/dev/dri:/dev/dri \
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
. install/setup.bash
```

5. Launch the simulator inside /workspace/sim
```
ros2 launch eufs_launcher eufs_launcher.launch.py
```

Common Erros
1. Visual Studio Code when trying to write to mounted file:
  Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
  Fix:
  ```sudo chown -R $USER .```

