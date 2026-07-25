# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This repo is the open-source project **Mimic** (`umersanii/Mimic` on GitHub). Reference/technical
documentation lives in a separate sibling repo, **Mimic-docs** (`umersanii/Mimic-docs`, MkDocs +
Material, at `/home/sani/c0d3/Mimic-docs` locally) — update it alongside code changes when behavior
documented there changes, not just this repo's own README.

## Learning goal for this project

The user's primary aim with this repo is to **maximize their own robotics learning**, not to have Claude Code drive raw output. They want to engage as the **architect/systems designer** of this project — understanding tradeoffs, making the calls, and building intuition for how the pieces (vision, control, firmware, simulation) fit together — rather than reading or reviewing implementation line by line.

Sessions in this repo should be steered accordingly:
- Explain the *why* behind architectural and design choices (e.g. why a given simulation approach, joint model, or control scheme), not just deliver working code.
- Default to surfacing decisions and tradeoffs to the user rather than silently picking one — this project is a vehicle for the user to practice making those calls.
- Favor conceptual/systems-level explanation over walking through code line by line, unless the user explicitly asks for a code-level review.
- It's fine to still write and run code — the point is to keep the user in the architect's seat and build their robotics understanding along the way, not to withhold implementation.

## Repository contents

This repository holds two unrelated things:

1. `isaac-sim-setup.md` — a runbook for running **NVIDIA Isaac Sim 5.0.0** headlessly via Docker with WebRTC streaming on this machine (Arch Linux, Lenovo Legion 5 Pro, RTX 3070 Ti).
2. A **webcam-controlled, tendon-driven robotic hand** project (inspired by pathofseb's build): `vision/` (MediaPipe hand tracking, Python), `firmware/hand_controller/` (Arduino sketch driving 5 tendon servos over serial), `hardware/BOM.md` (parts list, wiring, open build decisions), `sim/` (Gazebo simulation, see below). See the top-level `README.md` for the quick-start commands. Status: vision pipeline verified working (live webcam test), hardware not yet built — check `hardware/BOM.md` before ordering parts, since servo power supply is still an open decision.

### Simulation — now Gazebo, not Isaac Sim

The hand project's simulation work has moved to **Gazebo Sim (Harmonic) + ROS2 Jazzy**, in `sim/` — a separate track from the `isaac-sim-setup.md` runbook above (that doc is retained for reference/other uses, but the hand isn't simulated in Isaac Sim).

- `sim/docker/` — Dockerfile builds `robotics-gazebo` image (Ubuntu 24.04 + gz-harmonic + ROS2 Jazzy ros-base + ros_gz). `sim/docker/run.sh` mounts the whole `sim/` dir to `/sim` in the container (not just `worlds/`) and sets `GZ_SIM_RESOURCE_PATH=/sim/models`, since mesh files live under `sim/models/`.
- `sim/worlds/box_world.sdf` — primitive-shape joint/physics learning rig (pendulum, hinge, box), uses `dartsim`.
- `sim/worlds/hand_world.sdf` + `sim/models/hand/` — the InMoov-derived left-hand model (see below). **Uses `bullet-featherstone`, not `dartsim`** — see the mimic-joint constraint below for why.
- **`dartsim` (the physics engine `box_world.sdf` uses) silently ignores `<mimic>` joints** in this Gazebo build — logs "physics engine does not support mimic constraints" and just leaves the joint unconstrained. Verified `bullet-featherstone` applies mimic correctly (tested: commanding one driver joint moved all 3 mimic-coupled finger joints together). Any new world file that relies on mimic joints must set `type="bullet-featherstone"` and add `<engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>` inside the `Physics` system plugin — don't copy `box_world.sdf`'s dartsim config for hand/mimic-joint work without changing this.
- **Hand model provenance**: `sim/models/hand/hand.urdf` (and converted `hand.sdf`) were extracted from the left-hand subtree (`i01.leftHand.*` links/joints) of the whole-body URDF in [Sentience-Robotics/inmoov_ros_sim](https://github.com/Sentience-Robotics/inmoov_ros_sim) (`urdf/sentience_gz.urdf`), not written from scratch and not used as-is. Adaptations made:
  - **Source meshes/joint-origins were ~10x real scale** (measured STL bounding boxes: palm mesh was ~0.79×0.88×1.09 "meters" — an 88cm palm). All mesh `scale` attributes and origin translations (joint + visual) were multiplied by 0.1 to bring the palm to a real ~9-11cm size. Rotations were left untouched.
  - **Collapsed to 1 actuated DOF per finger** (matching the real hardware's 1-servo-per-finger tendon design) via `<mimic>` joints: one proximal joint per finger (`index_link_joint`, `majeure_link_joint`, `pinky_link_joint`, `ringFinger_link_joint`, `thumb1_link_joint`) is the driver; the other 2 joints per finger mimic it 1:1. The pinky/ring-finger abduction joints (`pinky0_link_joint`, `ringfinger0_link_joint`) were converted to `type="fixed"` (no spread motion — the real hardware doesn't have it either).
  - **Added `<collision>` (box primitives, sized from STL bounding boxes) and `<inertial>` (box-formula inertia, hand-estimated masses ~5-90g per link)** — the source URDF only had `<visual>` meshes, no collision/inertial data at all, so nothing would have had contact physics or dynamics without this.
  - The extraction/scaling/mimic/collision scripts used to produce `hand.urdf` were one-off Python (ElementTree) scripts, not checked into the repo — if the model needs regenerating (e.g. to also pull the right hand, or retune masses), redo from the source repo rather than assuming a saved script exists.
  - `sim/worlds/hand_world.sdf`'s model `<pose>` includes a 180° roll + 90° yaw correction — the source mesh assumes the hand hangs off an arm (fingers down at rest); mounting it straight to a fixed base with no arm chain left it in that "hanging" orientation, so the pose corrects it to a bench-top stance.

### `sim/bridge/gz_hand_bridge.py` — OpenCV vision → Gazebo hand bridge

Lets `vision/hand_tracker.py` (in the `robohand` conda env, host side) drive the simulated hand in real time, in parallel with (or instead of) the real Arduino over serial (`--gazebo` flag, `--no-serial` to skip the real hardware).

- **Why it's a separate process inside the container, not a library call**: `gz.transport13`/`gz.msgs10` Python bindings only exist inside the `robotics-gazebo` image (confirmed via `apt list --installed`); the host's `robohand` conda env has no path to them. `hand_tracker.py` launches it via `docker exec -i robotics_gazebo_sim python3 /sim/bridge/gz_hand_bridge.py` and streams one CSV line of 5 curl values (`thumb,index,middle,ring,pinky`, each 0.0=straight to 1.0=curled — same convention as `hand_tracker.py`'s own `curls` dict) per frame over that pipe. One persistent process/pipe, not a subprocess call per frame.
- `sim/docker/run.sh` sets `--name robotics_gazebo_sim` specifically so this bridge (and any other tooling) has a stable container name to target with `docker exec`.
- **Gotcha #1 — topic name must be the joint's last dot-segment, not the full URDF name.** `hand_world.sdf`'s `JointPositionController` plugins subscribe to e.g. `/inmoov_left_hand/index_link_joint/cmd_pos`, *not* `/inmoov_left_hand/i01.leftHand.index_link_joint/cmd_pos` — the world-generation script stripped the `i01.leftHand.` prefix when wiring up controller topics. The bridge script must do the same (`joint_name.split('.')[-1]`) or publishes silently go to a topic nobody's listening on.
- **Gotcha #2 — a single one-shot publish right after `node.advertise()` can be dropped.** Gazebo Transport discovery (multicast, finds existing subscribers) takes on the order of ~1s; publishing once immediately after advertising, then tearing the process down, can race past that window with nothing delivered. Not an issue for the real use case (continuous per-frame streaming for the whole session — discovery settles well within the first second and everything after that lands normally), but worth remembering if writing another one-shot test script against this bridge.

### Known issue: hand appeared to "jitter" — was rendering, not physics, now worked around

`hand_world.sdf`'s `sun` light has `<cast_shadows>false</cast_shadows>` — deliberately disabled, not an oversight. Symptom: the hand model visibly jittered/shimmered even completely at rest (no commands being sent at all). Diagnosed by subscribing directly to every joint's `axis1.position` at the sim's own ~1kHz update rate for 3s with zero commands published: every joint sat flat to within ~1e-8 rad — i.e. physics/joint state was provably static, so the jitter had to be a rendering artifact, not a control or physics instability.

Best-fit explanation (not filed upstream — a real Gazebo/gz-sim limitation, but this project just works around it locally): the default directional-light shadow-map resolution/depth-bias is tuned for room/building-scale scenes (meters). This hand is ~10cm with many small phalanx meshes packed close together, and at that scale the default shadow map produces shadow acne (flickering self-shadow noise) that reads visually as jitter. If shadows are ever turned back on for this world (e.g. for a nicer screenshot/demo), expect the jitter to return, and either re-disable them or hand-tune shadow map resolution/near-far clip/bias for cm-scale geometry.

### Robotic hand — environment facts to preserve

- Runs in a dedicated conda env, **`robohand`** (Python 3.11) — `mediapipe` has no build for Python 3.13, which is the system-level interpreter here. Always `conda activate robohand` before running anything in `vision/`.
- `mediapipe` 0.10.35 (the version conda installs) **removed the legacy `mp.solutions.hands` API entirely** — only `mediapipe.tasks.python.vision` (`HandLandmarker`) remains. `vision/hand_tracker.py` is written against that Tasks API, not the old one; don't "fix" it back to `mp.solutions`.
- The Tasks API requires an explicit model file, not bundled in the pip package: `vision/models/hand_landmarker.task`, downloaded from `storage.googleapis.com/mediapipe-models/hand_landmarker/...`. If it's missing, re-download rather than assuming the API can run without it (see README.md for the curl command).
- `HandLandmarksConnections.HAND_CONNECTIONS` yields `Connection(start, end)` namedtuples (single int indices), not pairs of connections — a plain list of landmarks, not the old `.landmark` sub-attribute.

## Environment facts to preserve

These constraints came from real troubleshooting and should not be re-derived or second-guessed without new evidence:

- **Working NVIDIA driver range is R570–R580** (specifically `nvidia-open-dkms` 580.119.02 is confirmed working). Driver 610+ segfaults inside Isaac Sim's bundled RTX libraries (`librtx.scenedb.plugin.so`, `carbOnPluginStartup`) because its ABI is incompatible. Driver 565.x fails to build against kernel ≥6.12 (missing `phys_to_dma`, `dma_is_direct`, `ioremap_driver_hardened_wc`).
- **Isaac Sim 4.5.0 does not work** on this setup (requires driver ≤565, which conflicts with the kernel constraint above). **5.1.0-rc.19 segfaults** (unstable RC). Only **5.0.0** is confirmed stable.
- The **LTS kernel and the NVIDIA driver packages are both pinned** via `IgnorePkg` in `/etc/pacman.conf` to prevent pacman from silently upgrading past the working versions.
- Docker's default runtime must be set to `nvidia` in `/etc/docker/daemon.json` for `--runtime=nvidia` container GPU access to work.
- `--network=host` is required on the `docker run` for the WebRTC streaming client to reach port 49100; the media stream itself uses UDP port 47998.
- Only one WebRTC streaming client can connect at a time; NVENC (present on RTX, absent on A100) is required for streaming.
- ROS2 Humble is bundled inside the `nvcr.io/nvidia/isaac-sim:5.0.0` image — no host ROS2 install is needed.
- If the pinned R580 driver ever breaks, the documented fallback is R570 (`570.153.02-1`), from the same Arch Linux archive.

## Shell conventions

All example commands in this repo's docs are written for **fish shell** (not bash) — e.g. `set VAR value` instead of `export VAR=value`, `set -a pkgs ...` for array append. Keep this convention when adding or editing commands in `isaac-sim-setup.md` or elsewhere in this repo.

## Common commands (from isaac-sim-setup.md)

Pull and run Isaac Sim headless with WebRTC streaming:
```fish
docker run --rm --runtime=nvidia --network=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y nvcr.io/nvidia/isaac-sim:5.0.0 ./runheadless.native.sh
```

Run a standalone Python script without a streaming viewer:
```fish
docker run --rm --runtime=nvidia -e ACCEPT_EULA=Y nvcr.io/nvidia/isaac-sim:5.0.0 ./python.sh /isaac-sim/standalone_examples/api/omni.isaac.core/hello_world.py
```

Verify the active driver version:
```fish
nvidia-smi
```
