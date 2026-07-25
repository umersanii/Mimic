"""
Runs INSIDE the robotics-gazebo container (needs its gz.transport13 / gz.msgs10
Python bindings, not available in the host's robohand conda env).

Reads one CSV line per frame from stdin: "thumb,index,middle,ring,pinky" as
curl fractions in [0.0, 1.0] (0 = straight, 1 = fully curled - same convention
as vision/hand_tracker.py's `curls` dict), scales each to the driver joint's
[0, 1.5708] rad range, and publishes to that finger's cmd_pos topic via
persistent publishers (one advertise per topic, many publishes - avoids the
per-call advertise/discover overhead of shelling out to `gz topic pub`).

Launched by vision/hand_tracker.py via `docker exec -i <container> python3
/sim/bridge/gz_hand_bridge.py` - stdin is the pipe between them.
"""

import sys

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

JOINT_NAMES = {
    "thumb": "i01.leftHand.thumb1_link_joint",
    "index": "i01.leftHand.index_link_joint",
    "middle": "i01.leftHand.majeure_link_joint",
    "ring": "i01.leftHand.ringFinger_link_joint",
    "pinky": "i01.leftHand.pinky_link_joint",
}

JOINT_MAX_RAD = 1.5708  # matches the <limit upper=...> baked into hand.urdf


def main():
    node = Node()
    # Topic uses only the joint name's last dot-segment (e.g. "index_link_joint"),
    # matching the JointPositionController <topic> baked into hand_world.sdf -
    # NOT the full "i01.leftHand.index_link_joint" joint identifier.
    publishers = {
        finger: node.advertise(f"/inmoov_left_hand/{joint.split('.')[-1]}/cmd_pos", Double)
        for finger, joint in JOINT_NAMES.items()
    }

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            values = [float(v) for v in line.split(",")]
        except ValueError:
            continue
        if len(values) != len(FINGER_ORDER):
            continue

        for finger, curl in zip(FINGER_ORDER, values):
            curl = max(0.0, min(1.0, curl))
            msg = Double()
            msg.data = curl * JOINT_MAX_RAD
            publishers[finger].publish(msg)


if __name__ == "__main__":
    main()
