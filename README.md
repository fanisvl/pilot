# autopilot

1. Clone the repo

3. Build image
  ```docker build -t nvidia_ros .```

4. Run container - This will also mount the repo you cloned to the docker container

  (no-gpu)
  ```
  docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v .:/workspace/autopilot \
    nvidia_ros \
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
      nvidia_ros \
      bash
  ```
