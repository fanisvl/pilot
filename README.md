# autopilot

1. Clone the repo

3. Download (https://github.com/AUEB-CS-Autonomous-Racing/autopilot/pkgs/container/autopilot%2Fautopilot) or build image
  ```docker build -t pilot .```

4. Run container - This will also mount the repo you cloned to the docker container

Note: These commands should be ran inside /autopilot  
  (no-gpu)
  ```
  docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v .:/workspace/autopilot \
    pilot \
    bash
  ```

(gpu - jetson) -> not tested
  ```
  docker run -it --net=host --gpus all \
      --env="NVIDIA_DRIVER_CAPABILITIES=all" \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      -v .:/workspace/autopilot \
      pilot \
      bash
  ```


Common Erros
1. Visual Studio Code when trying to write to mounted file:
  Failed to save Insufficient permissions. Select 'Retry as Sudo' to retry as superuser.
  Fix:
  ```sudo chown -R $USER .```
