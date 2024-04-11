# autopilot

1. Clone the repo

3. Build image
  ```docker build -t nvidia_ros .```

4. Run container
  (jetson)
  ```
  docker run -it --net=host --gpus all \
      --env="NVIDIA_DRIVER_CAPABILITIES=all" \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      nvidia_ros \
      bash
  ```
  (no-gpu)
  ```
  docker run -it --net=host  \          
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      nvidia_ros \
      bash
  ```
