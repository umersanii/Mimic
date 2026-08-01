#!/usr/bin/env fish
# Run Gazebo Sim in Docker with GPU rendering, GUI forwarded over X11.
# Same machine as isaac-sim-setup.md, so X11 forwarding is simpler than WebRTC.

xhost +local:docker

docker run --rm \
    --name robotics_gazebo_sim \
    --runtime=nvidia \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -e QT_SCALE_FACTOR=1.25 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v (dirname (status --current-filename))/..:/sim \
    -e GZ_SIM_RESOURCE_PATH=/sim/models \
    robotics-gazebo \
    $argv
