"""
Runs INSIDE the robotics-gazebo container (needs its gz.transport13 / gz.msgs10
Python bindings, not available in the host's robohand conda env).

Reads one CSV line per frame from stdin: "L,thumb,index,middle,ring,pinky" or
"R,thumb,index,middle,ring,pinky" - a side label plus curl fractions in [0.0, 1.0]
(0 = straight, 1 = fully curled - same convention as vision/hand_tracker.py's `curls`
dict), scales each to the driver joint's [0, 1.5708] rad range, and publishes to
that side+finger's cmd_pos topic via persistent publishers (one advertise per topic,
many publishes - avoids the per-call advertise/discover overhead of shelling out to
`gz topic pub`).

Sends one line per currently-tracked hand per frame, not necessarily both every
frame - see hand_tracker.py, which holds last-known curls for a hand that drops out
of the webcam view rather than snapping it to a default pose.

The wrist joint exists per hand in hand_world.sdf but has no publisher here - not
vision-driven yet (2D hand landmarks can't reliably estimate forearm roll without
the forearm in view). Command it manually via `gz topic pub` for testing.

Launched by vision/hand_tracker.py via `docker exec -i <container> python3
/sim/bridge/gz_hand_bridge.py` - stdin is the pipe between them.
"""

import sys

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

# Driver joint names match hand.urdf: the single tendon-actuated joint per finger
# (always the one directly off hand_link) that the other 2 (or 3, for ring/pinky)
# segments mimic - see sim/models/hand/generate_hand.py.
JOINT_NAMES = {
    "thumb": "thumb1_joint",
    "index": "index1_joint",
    "middle": "middle1_joint",
    "ring": "ring1_joint",
    "pinky": "pinky1_joint",
}

MODEL_NAMES = {"L": "inmoov_left_hand", "R": "inmoov_right_hand"}

JOINT_MAX_RAD = 1.5708  # matches the <limit upper=...> baked into hand.urdf


def main():
    node = Node()
    publishers = {
        side: {
            finger: node.advertise(f"/{model}/{joint}/cmd_pos", Double)
            for finger, joint in JOINT_NAMES.items()
        }
        for side, model in MODEL_NAMES.items()
    }

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) != len(FINGER_ORDER) + 1:
            continue
        side, raw_values = parts[0], parts[1:]
        if side not in publishers:
            continue
        try:
            values = [float(v) for v in raw_values]
        except ValueError:
            continue

        for finger, curl in zip(FINGER_ORDER, values):
            curl = max(0.0, min(1.0, curl))
            msg = Double()
            msg.data = curl * JOINT_MAX_RAD
            publishers[side][finger].publish(msg)


if __name__ == "__main__":
    main()
