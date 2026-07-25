"""
Webcam -> MediaPipe hand landmarks -> per-finger flexion angles -> serial bytes to the
Arduino running firmware/hand_controller, which drives the tendon servos.

Run: python3 hand_tracker.py [--port /dev/ttyUSB0] [--no-serial]

Uses the MediaPipe Tasks API (mediapipe>=0.10.x dropped the old mp.solutions.hands API),
which requires the hand_landmarker.task model file in models/ - see README.md to fetch it.
"""

import argparse
import os
import subprocess
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    HandLandmarker,
    HandLandmarkerOptions,
    HandLandmarksConnections,
    RunningMode,
)

import hud

try:
    import serial
except ImportError:
    serial = None

MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "hand_landmarker.task")

# landmark indices per finger: (mcp, pip, tip) - used to estimate flexion angle
FINGER_JOINTS = {
    "thumb": (2, 3, 4),
    "index": (5, 6, 8),
    "middle": (9, 10, 12),
    "ring": (13, 14, 16),
    "pinky": (17, 18, 20),
}
FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]


class HandDetector:
    def __init__(self, model_path=MODEL_PATH, max_hands=1, detection_conf=0.7, tracking_conf=0.7):
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf,
        )
        self.landmarker = HandLandmarker.create_from_options(options)
        self.landmarks = None
        self._start_time = time.time()

    def find_hands(self, frame, draw=True):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.time() - self._start_time) * 1000)
        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        self.landmarks = None
        if result.hand_landmarks:
            self.landmarks = result.hand_landmarks[0]
            if draw:
                self._draw_landmarks(frame)
        return frame

    def _draw_landmarks(self, frame):
        h, w = frame.shape[:2]
        points = [(int(lm.x * w), int(lm.y * h)) for lm in self.landmarks]
        hud.draw_skeleton(frame, points, HandLandmarksConnections.HAND_CONNECTIONS)

    def _xy(self, idx, w, h):
        lm = self.landmarks[idx]
        return np.array([lm.x * w, lm.y * h])

    def get_angles(self, frame_shape):
        """Return {finger_name: angle_degrees} - 180 = straight, smaller = curled."""
        if self.landmarks is None:
            return None
        h, w = frame_shape[:2]
        angles = {}
        for name, (mcp_i, pip_i, tip_i) in FINGER_JOINTS.items():
            wrist = self._xy(0, w, h)
            mcp = self._xy(mcp_i, w, h)
            tip = self._xy(tip_i, w, h)
            a = wrist if name != "thumb" else self._xy(1, w, h)
            b = mcp
            c = tip
            ba = a - b
            bc = c - b
            cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
            angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
            angles[name] = angle
        return angles

    def count_extended(self, angles, threshold=110):
        if angles is None:
            return 0
        return sum(1 for a in angles.values() if a > threshold)


def curl_fraction(angle, straight=170, curled=40):
    """0.0 = fully curled, 1.0 = fully straight."""
    t = (angle - curled) / (straight - curled)
    return float(np.clip(t, 0.0, 1.0))


def angle_to_servo(angle, servo_open=180, servo_closed=0):
    """Map a measured joint angle to a servo command (0-180)."""
    t = curl_fraction(angle)
    return int(servo_closed + t * (servo_open - servo_closed))


def open_serial(port, baud=115200):
    if serial is None:
        raise RuntimeError("pyserial not installed - pip install pyserial, or run with --no-serial")
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(2)  # Arduino resets on serial open
    return ser


def open_gazebo_bridge(container="robotics_gazebo_sim"):
    """Start sim/bridge/gz_hand_bridge.py inside the running Gazebo container.

    That script needs gz.transport13/gz.msgs10, which only exist inside the
    robotics-gazebo image - not the host's robohand conda env - so it runs
    over there and this process just streams CSV curl values to its stdin
    via `docker exec -i`, one persistent pipe instead of a subprocess call
    per frame.
    """
    return subprocess.Popen(
        ["docker", "exec", "-i", container, "python3", "/sim/bridge/gz_hand_bridge.py"],
        stdin=subprocess.PIPE,
        text=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for the hand's Arduino")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--no-serial", action="store_true", help="Run vision only, skip serial output")
    parser.add_argument("--gazebo", action="store_true", help="Also drive the Gazebo sim hand")
    parser.add_argument("--gazebo-container", default="robotics_gazebo_sim",
                         help="Name of the running robotics-gazebo container (see sim/docker/run.sh)")
    parser.add_argument("--smoothing", type=float, default=0.3,
                         help="EMA weight on each new curl reading, 0-1. Lower = smoother/laggier, "
                              "higher = snappier/jitterier. Raw MediaPipe landmarks jitter a bit even "
                              "for a held-still hand; this filters that out before anything is sent "
                              "downstream (serial or Gazebo).")
    args = parser.parse_args()

    ser = None if args.no_serial else open_serial(args.port, args.baud)
    serial_state = "disabled" if ser is None else "connected"

    gz_bridge = open_gazebo_bridge(args.gazebo_container) if args.gazebo else None
    gazebo_state = "disabled" if gz_bridge is None else "connected"

    cap = cv2.VideoCapture(args.camera)
    detector = HandDetector()
    prev_time = time.time()
    fps_smoothed = 0.0
    smoothed_angles = None  # EMA state, carried across frames to damp landmark jitter

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)
        frame = detector.find_hands(frame)
        angles = detector.get_angles(frame.shape)

        if angles is not None:
            if smoothed_angles is None:
                smoothed_angles = dict(angles)
            else:
                a = args.smoothing
                smoothed_angles = {
                    f: a * angles[f] + (1 - a) * smoothed_angles[f] for f in FINGER_ORDER
                }
            angles = smoothed_angles
        else:
            smoothed_angles = None  # hand lost - don't smooth into a stale pose on reacquire

        now = time.time()
        fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
        prev_time = now
        fps_smoothed = fps if fps_smoothed == 0.0 else fps_smoothed * 0.9 + fps * 0.1

        curls = None
        if angles is not None:
            curls = {f: 1.0 - curl_fraction(angles[f]) for f in FINGER_ORDER}
            servo_cmds = [angle_to_servo(angles[f]) for f in FINGER_ORDER]
            if ser is not None:
                try:
                    line = ",".join(str(v) for v in servo_cmds) + "\n"
                    ser.write(line.encode("ascii"))
                    serial_state = "connected"
                except serial.SerialException:
                    serial_state = "disconnected"
            if gz_bridge is not None:
                try:
                    line = ",".join(str(curls[f]) for f in FINGER_ORDER) + "\n"
                    gz_bridge.stdin.write(line)
                    gz_bridge.stdin.flush()
                    gazebo_state = "connected"
                except (BrokenPipeError, OSError):
                    gazebo_state = "disconnected"
            finger_count = detector.count_extended(angles)
        else:
            finger_count = 0

        hud.draw_header(frame, fps_smoothed)
        hud.draw_serial_status(frame, serial_state)
        if gz_bridge is not None or args.gazebo:
            hud.draw_gazebo_status(frame, gazebo_state)
        hud.draw_finger_gauges(frame, curls, FINGER_ORDER, finger_count)
        cv2.imshow("Robotic Hand Control", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()
    if gz_bridge is not None:
        gz_bridge.stdin.close()
        gz_bridge.terminate()


if __name__ == "__main__":
    main()
