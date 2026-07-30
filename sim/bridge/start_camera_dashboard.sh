#!/usr/bin/env fish
# Launch gz_camera_dashboard.py detached inside the running robotics-gazebo container.
# Run once after the container + `gz sim hand_world.sdf` are already up.

docker exec -d robotics_gazebo_sim python3 /sim/bridge/gz_camera_dashboard.py
