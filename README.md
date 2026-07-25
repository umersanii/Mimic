# Mimic

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Mimic** — a webcam-controlled, tendon-driven robotic hand. MediaPipe hand tracking on a PC reads
finger flexion in real time; today that drives a simulated hand in Gazebo, and will drive real
servos on a 3D-printed hand over serial once the physical build exists.
(Inspired by pathofseb's build.)

Open source, hobbyist-built, and documented as a learning project — see
[Documentation](#documentation) below for the full technical + conceptual writeup.

**Simulation-based for now.** This repo currently covers the vision pipeline and the Gazebo
simulation only. Firmware and hardware are designed and planned but intentionally not part of this
repo yet — they'll be added once the physical hand is actually built and wired up. See
[Planned: hardware & firmware](#planned-hardware--firmware) below.

## Layout

- `vision/` — Python: webcam capture, MediaPipe hand landmarks, per-finger flexion angle
  calculation, serial output (once hardware exists) and Gazebo bridge output. Entry point:
  `vision/hand_tracker.py`.
- `sim/` — Gazebo simulation (world, InMoov-derived hand model, vision→sim bridge). See the docs
  site for the full writeup of how it works.
- `isaac-sim-setup.md` — unrelated: Isaac Sim 5.0.0 headless Docker setup notes for this machine.

## Status

Vision pipeline and simulation are working (live-tested against the webcam; sim verified with the
same tracked hand data). Physical hardware is not built — this repo will grow a `firmware/` and
`hardware/` directory once it is.

## Quick start (software side, no hardware needed yet)

Uses a dedicated conda env (`robohand`, Python 3.11) — `mediapipe` doesn't ship for 3.13, which
is what's installed at the system level.

```fish
conda create -y -n robohand python=3.11
conda activate robohand
cd vision
pip install -r requirements.txt
python3 hand_tracker.py --no-serial
```

This opens a webcam window with hand-skeleton overlay, finger count, and FPS — the same
debug view as the reference video — without needing the Arduino/servos connected.

### Model file

`hand_tracker.py` uses MediaPipe's newer Tasks API (`mediapipe>=0.10.x` dropped the old
`mp.solutions.hands` API), which needs an explicit model file rather than one bundled in the
package. Already downloaded to `vision/models/hand_landmarker.task`. If it's ever missing:

```fish
mkdir -p vision/models
curl -sL -o vision/models/hand_landmarker.task \
  "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
```

### Drive the simulated hand instead

With the [Gazebo simulation](https://github.com/umersanii/Mimic-docs) set up:

```fish
python3 hand_tracker.py --no-serial --gazebo
```

## Planned: hardware & firmware

The physical build isn't started yet, but it's fully planned: a tendon-driven hand (1 servo per
finger, tendons routed through printed channels, elastic return), an Arduino reading finger angles
over serial and driving the servos, and a bill of materials with the open decisions already
identified (which hand STL to print, servo power supply sizing). That plan — the BOM, wiring
notes, and firmware design — is written up in full on the [docs site](https://github.com/umersanii/Mimic-docs),
which is where it'll stay linked from once `firmware/` and `hardware/` land in this repo.

## Documentation

This README covers quick-start commands. The full reference documentation — architecture,
how the vision-to-servo pipeline works, the simulation setup, and build/wiring notes, written
for both newcomers and anyone who wants the technical detail — lives in a separate docs repo:

**[umersanii/Mimic-docs](https://github.com/umersanii/Mimic-docs)** (deploys to GitHub Pages)

## Contributing

Contributions and forks are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MIT](LICENSE)
