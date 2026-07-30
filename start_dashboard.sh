#!/usr/bin/env fish
# One-shot launcher for the combined vision+sim web dashboard: starts (or reuses) the
# Gazebo container + world, starts the sim camera bridge, then starts the vision
# dashboard - opens http://localhost:8765/ showing both the webcam and sim feeds.
#
# Defaults to fully headless: sim runs with -s -r --headless-rendering (server-only,
# offscreen sensor rendering, no X11/GPU-passthrough window) and hand_tracker.py runs
# without its local cv2 window - only the browser dashboard at localhost is a GUI. Pass
# --gui to get both local windows back (the gz sim GUI window - only takes effect when
# this script is the one starting the container, no effect if robotics_gazebo_sim is
# already running and reused as-is - and hand_tracker.py's cv2 window, via --window).
#
# Remaining args are passed through to hand_tracker.py, e.g.:
#   ./start_dashboard.sh --gazebo
#   ./start_dashboard.sh --gui --gazebo
#
# Ctrl+C stops the vision dashboard (it runs in the foreground). The Gazebo
# container and camera bridge are left running so you don't lose your sim
# session - rerun this script any time to reattach.

set REPO_ROOT (dirname (status --current-filename))

set GUI_MODE 0
set REMAINING_ARGS
for a in $argv
    if test "$a" = "--gui"
        set GUI_MODE 1
    else
        set -a REMAINING_ARGS $a
    end
end

if not docker ps --filter name=robotics_gazebo_sim --format '{{.Names}}' | grep -q robotics_gazebo_sim
    if test $GUI_MODE -eq 1
        echo "Starting robotics_gazebo_sim container (GUI)..."
        $REPO_ROOT/sim/docker/run.sh gz sim /sim/worlds/hand_world.sdf &
    else
        echo "Starting robotics_gazebo_sim container (headless)..."
        $REPO_ROOT/sim/docker/run.sh gz sim -s -r --headless-rendering /sim/worlds/hand_world.sdf &
    end
    for i in (seq 30)
        if docker ps --filter name=robotics_gazebo_sim --format '{{.Names}}' | grep -q robotics_gazebo_sim
            break
        end
        sleep 1
    end
else
    echo "robotics_gazebo_sim already running, reusing it (--gui has no effect here)."
end

echo "Waiting for the overview camera sensor to publish..."
for i in (seq 30)
    if docker exec robotics_gazebo_sim bash -c "source /opt/ros/jazzy/setup.bash; gz topic -l 2>/dev/null" | grep -q overview_camera/image
        break
    end
    sleep 1
end

if not docker exec robotics_gazebo_sim pgrep -f gz_camera_dashboard.py >/dev/null 2>&1
    echo "Starting sim camera bridge..."
    $REPO_ROOT/sim/bridge/start_camera_dashboard.sh
    sleep 1
else
    echo "Camera bridge already running."
end

if test $GUI_MODE -eq 1
    set -a REMAINING_ARGS --window
end

echo "Starting vision dashboard at http://localhost:8765/ ..."
cd $REPO_ROOT/vision
conda run -n robohand --no-capture-output python3 hand_tracker.py --no-serial --dashboard $REMAINING_ARGS
