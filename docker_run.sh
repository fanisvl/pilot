sudo docker run -it --name autopilot \
  --runtime nvidia \
  --net=host \
  -e DISPLAY=$DISPLAY\
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /etc/enctune.conf:/etc/enctune.conf \
  -v /etc/nv_tegra_release:/etc/nv_tegra_release \
  -v /var/run/dbus:/var/run/dbus \
  -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
  -v ~/workspace/pilot:/workspace/pilot \
  -v ~/workspace/jetson-inference:/workspace/jetson-inference \
  -w /workspace \
  fanisvl/autopilot:latest
