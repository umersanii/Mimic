#!/usr/bin/env python3
"""Generates hand.urdf for sim/models/hand/{left,right}/ from the MyRobotLab/inmoov_ros
mesh set copied into meshes/. See CLAUDE.md for full provenance/licensing notes.

Joint topology (link offsets, mimic wiring, axes) is transcribed from that repo's
inmoov_description/urdf/asmHand.urdf.xacro and asmArm/inmoov.urdf.xacro (wrist_roll_joint),
resolved by hand for side="l" (flip=-1) / side="r" (flip=1) instead of running the real
xacro toolchain (which needs $(find inmoov_bringup)/config/config.yaml via ROS package
resolution - not worth standing up a colcon workspace for a one-time generation step).

Finger joint limits/velocity intentionally do NOT reuse that repo's config.yaml minGoal/
maxGoal (real per-servo calibration in degrees, not meaningful here) - kept at this
project's existing 0..1.5708 rad / 0.7854 rad/s convention instead, matching the old
hand.urdf and sim/bridge/gz_hand_bridge.py's JOINT_MAX_RAD scaling.

Inertial values also do NOT reuse that repo's config.inertial.urdf.xacro macros - their
mass units are ambiguous (raw/1000 lands in a "grams-as-kg-looking" range with no unit
comment) and not worth trusting unverified. Collision boxes are computed from each mesh's
own bounding box; masses are hand-estimated by link role, same method CLAUDE.md documents
for the previous model (box-formula inertia from an assumed mass, not measured).

Visual material/color is intentionally NOT set here even though URDF supports it -
`gz sdf -p` silently drops URDF <material><color/></material> during URDF->SDF
conversion (every visual comes out plain white regardless). build_world.py patches
the material onto the converted SDF directly instead - see its docstring.

Run: python3 generate_hand.py, then (inside the robotics-gazebo container, needs
`gz`) gz sdf -p left/hand.urdf > left/hand.sdf (and right/), then build_world.py.
"""

import struct
from pathlib import Path

HERE = Path(__file__).parent
MESH_SCALE = 0.001  # source STLs are in millimeters

FINGER_JOINT_LOWER = 0.0
FINGER_JOINT_UPPER = 1.5708
FINGER_JOINT_VELOCITY = 0.7854
FINGER_JOINT_EFFORT_DRIVER = 2.0
FINGER_JOINT_EFFORT_MIMIC = 0.0

WRIST_VELOCITY = 1.0
WRIST_EFFORT = 2.0

# mass (kg) by structural role, matching the ~0.005-0.09 kg per-link range CLAUDE.md
# documents for the previous model
MASS = {
    "hand": 0.080,
    "forearm": 0.090,
    "proximal": 0.012,   # first phalanx off the palm
    "mid": 0.008,
    "mid2": 0.006,       # ring/pinky's extra 3rd segment
    "distal": 0.005,
}


def read_stl_bounds(path):
    """Binary STL: 80-byte header, uint32 triangle count, 50 bytes/triangle
    (12 floats: normal xyz + 3 vertex xyz, then 2-byte attribute count)."""
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0]
    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3
    off = 84
    for _ in range(n):
        for v in range(3):
            vx, vy, vz = struct.unpack_from("<fff", data, off + 12 + v * 12)
            mins[0] = min(mins[0], vx); maxs[0] = max(maxs[0], vx)
            mins[1] = min(mins[1], vy); maxs[1] = max(maxs[1], vy)
            mins[2] = min(mins[2], vz); maxs[2] = max(maxs[2], vz)
        off += 50
    size = [(maxs[i] - mins[i]) * MESH_SCALE for i in range(3)]
    center = [((maxs[i] + mins[i]) / 2.0) * MESH_SCALE for i in range(3)]
    return size, center


def box_inertia(mass, size):
    a, b, c = size
    a = max(a, 1e-6); b = max(b, 1e-6); c = max(c, 1e-6)
    return (
        mass / 12.0 * (b * b + c * c),
        mass / 12.0 * (a * a + c * c),
        mass / 12.0 * (a * a + b * b),
    )


def link_xml(name, mesh_file, mesh_dir, mass_key):
    size, center = read_stl_bounds(mesh_dir / mesh_file)
    mass = MASS[mass_key]
    ixx, iyy, izz = box_inertia(mass, size)
    cx, cy, cz = center
    sx, sy, sz = size
    return f"""  <link name="{name}">
    <visual name="{name}_visual">
      <geometry>
        <mesh filename="meshes/{mesh_file}" scale="{MESH_SCALE} {MESH_SCALE} {MESH_SCALE}" />
      </geometry>
    </visual>
    <collision name="{name}_collision">
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0" />
      <geometry>
        <box size="{sx:.6f} {sy:.6f} {sz:.6f}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0" />
      <mass value="{mass:.6f}" />
      <inertia ixx="{ixx:.9f}" ixy="0" ixz="0" iyy="{iyy:.9f}" iyz="0" izz="{izz:.9f}" />
    </inertial>
  </link>
"""


def joint_xml(name, jtype, parent, child, xyz, rpy, axis=None, mimic=None,
              lower=FINGER_JOINT_LOWER, upper=FINGER_JOINT_UPPER,
              velocity=FINGER_JOINT_VELOCITY, effort=FINGER_JOINT_EFFORT_MIMIC):
    x, y, z = xyz
    r, p, yaw = rpy
    parts = [f'  <joint name="{name}" type="{jtype}">']
    parts.append(f'    <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{r:.6f} {p:.6f} {yaw:.6f}" />')
    parts.append(f'    <parent link="{parent}" />')
    parts.append(f'    <child link="{child}" />')
    if jtype == "revolute":
        ax, ay, az = axis
        parts.append(f'    <axis xyz="{ax:.6f} {ay:.6f} {az:.6f}" />')
        parts.append(f'    <limit lower="{lower:.6f}" upper="{upper:.6f}" effort="{effort:.6f}" velocity="{velocity:.6f}" />')
        if mimic:
            mjoint, mult = mimic
            parts.append(f'    <mimic joint="{mjoint}" multiplier="{mult:.6f}" offset="0.0" />')
    parts.append("  </joint>")
    return "\n".join(parts) + "\n"


# (name_suffix, mesh_side_specific, mass_key, parent_suffix or None-for-hand,
#  joint_type, origin_xyz(flip applied via lambda), origin_rpy(flip applied via lambda),
#  axis, driver_joint_name or None if this IS the driver, mimic_multiplier)
def finger_chain(side, flip):
    """Returns list of (link_name, mesh_file, mass_key) and
    list of (joint_name, parent, child, xyz, rpy, axis, mimic_or_None)."""
    links = []
    joints = []

    def L(name, mesh, mass_key):
        links.append((name, mesh, mass_key))

    def J(name, parent, child, xyz, rpy, axis, mimic=None):
        joints.append((name, parent, child, xyz, rpy, axis, mimic))

    s = side  # "l" or "r"
    f = flip  # -1.0 or 1.0

    # Driver is always the joint directly off hand_link (tree-proximal), every other
    # joint in the chain mimics it 1:1 - required by bullet-featherstone, which only
    # actuates a driver correctly when mimic joints are all *downstream* of it in the
    # kinematic tree (confirmed by testing: a driver with an upstream/ancestor mimic
    # joint - this source repo's original design, where e.g. index_joint drove and
    # index1_joint upstream of it mimicked - silently produced zero torque response).
    # This also means the coupling ratios this source repo used (e.g. thumb1 = 0.75 *
    # thumb, ring1 = -0.1 * ring) are dropped in favor of a flat 1:1 mimic on every
    # joint in a chain - those ratios were tuned for the *other* joint being the
    # driver and don't carry over; simple 1:1 still gives a coordinated curl, just
    # not a bit-for-bit reproduction of the source robot's exact coupling.

    # THUMB - driver is thumb1_joint (hand_link -> thumb1_link)
    L("thumb1_link", f"{s}_thumb5_1.stl", "proximal")
    L("thumb2_link", "thumb5_2.stl", "mid")
    L("thumb3_link", "thumb5_3.stl", "distal")
    # Axis is NOT a simple X/Y/Z: in the original inmoov_ros source xacro, thumb1_joint
    # was never a driver at all - it was a passive mimic of thumb_joint (multiplier
    # flip*0.75), with its own axis (0,0,1) authored only for that small secondary
    # coupling. The real flexion motion lived on thumb_joint (axis (0,1,0)), one joint
    # further down the chain, in a much more heavily-tilted frame (rpy 0.825,-0.1,0.3
    # vs thumb1_joint's own 0.1,0,0). Promoting thumb1_joint to driver (required by
    # bullet-featherstone - see the module comment above) is harmless for index/middle/
    # ring/pinky, whose proximal joint's own axis already matches their real driver's
    # axis/plane. It is NOT harmless for the thumb: thumb1_joint's native axis is a
    # different rotation plane entirely from thumb_joint's, so driving it directly
    # produces some other motion (spin/sideways sweep depending on axis choice tried),
    # never the real curl - confirmed by testing plain X/Y/Z guesses on thumb1_joint,
    # all wrong in different ways.
    # Fix: re-express thumb_joint's own axis (0,1,0), rotated by thumb_joint's own rpy
    # (flip*0.825, -0.1, flip*0.3), one frame up into thumb1_joint's frame - thumb1_joint's
    # own rpy cancels out of this since we're solving for "same axis, one joint up the
    # chain," not "same axis in hand_link's frame." R = Rz(yaw)*Ry(pitch)*Rx(roll) applied
    # to (0,1,0), computed with numpy, gives (-0.27058443*f, 0.62657902, 0.7308781*f) -
    # confirmed correct by direct gz topic pub testing (curls into the palm, not a spin
    # or sideways sweep).
    J("thumb1_joint", "hand_link", "thumb1_link",
      (0.0, f * 0.029, -0.0577), (f * 0.1, 0.0, 0.0),
      (-0.27058443 * f, 0.62657902, 0.7308781 * f))
    J("thumb_joint", "thumb1_link", "thumb2_link",
      (-0.00052, f * 0.02725, -0.013), (f * 0.825, -0.1, f * 0.3), (0.0, 1.0, 0.0),
      mimic=("thumb1_joint", 1.0))
    J("thumb3_joint", "thumb2_link", "thumb3_link",
      (0.0, 0.0, -0.035), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("thumb1_joint", 1.0))

    # INDEX - driver is index1_joint (hand_link -> index1_link)
    L("index1_link", "index3_1.stl", "proximal")
    L("index2_link", "index3_2.stl", "mid")
    L("index3_link", "index3_3.stl", "distal")
    J("index1_joint", "hand_link", "index1_link",
      (-0.0015, f * 0.0342, -0.119), (f * 0.1, 0.0, 0.0), (0.0, 1.0, 0.0))
    J("index_joint", "index1_link", "index2_link",
      (0.0, 0.001, -0.03595), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("index1_joint", 1.0))
    J("index3_joint", "index2_link", "index3_link",
      (0.0, 0.0, -0.024), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("index1_joint", 1.0))

    # MIDDLE - driver is middle1_joint (hand_link -> middle1_link)
    L("middle1_link", "middle3_1.stl", "proximal")
    L("middle2_link", "middle3_2.stl", "mid")
    L("middle3_link", "middle3_3.stl", "distal")
    J("middle1_joint", "hand_link", "middle1_link",
      (-0.00175, f * 0.007, -0.12325), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    J("middle_joint", "middle1_link", "middle2_link",
      (0.0, 0.0, -0.0389), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("middle1_joint", 1.0))
    J("middle3_joint", "middle2_link", "middle3_link",
      (0.0, 0.0005, -0.0259), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("middle1_joint", 1.0))

    # RING - 4 segments, driver is ring1_joint (hand_link -> ring1_link)
    L("ring1_link", f"{s}_ring3_1.stl", "proximal")
    L("ring2_link", "ring3_2.stl", "mid")
    L("ring3_link", "ring3_3.stl", "mid2")
    L("ring4_link", "ring3_4.stl", "distal")
    J("ring1_joint", "hand_link", "ring1_link",
      (0.0, f * -0.00705, -0.0794), (f * 0.7, 0.0, 0.0), (0.0, 0.0, -f))
    J("ring_joint", "ring1_link", "ring2_link",
      (0.00126, f * -0.0351, -0.0166), (f * -0.7775, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("ring1_joint", 1.0))
    J("ring3_joint", "ring2_link", "ring3_link",
      (0.0, 0.0005, -0.0345), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("ring1_joint", 1.0))
    J("ring4_joint", "ring3_link", "ring4_link",
      (0.0, 0.0004, -0.0229), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("ring1_joint", 1.0))

    # PINKY - 4 segments, driver is pinky1_joint (hand_link -> pinky1_link)
    L("pinky1_link", f"{s}_pinky3_1.stl", "proximal")
    L("pinky2_link", "pinky3_2.stl", "mid")
    L("pinky3_link", "pinky3_3.stl", "mid2")
    L("pinky4_link", "pinky3_4.stl", "distal")
    J("pinky1_joint", "hand_link", "pinky1_link",
      (0.0, f * -0.0270, -0.0555), (f * 0.7, 0.0, 0.0), (0.0, 0.0, -f))
    J("pinky_joint", "pinky1_link", "pinky2_link",
      (0.0, f * -0.046, -0.0228), (f * -0.93, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("pinky1_joint", 1.0))
    J("pinky3_joint", "pinky2_link", "pinky3_link",
      (0.0, 0.0005, -0.031), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("pinky1_joint", 1.0))
    J("pinky4_joint", "pinky3_link", "pinky4_link",
      (0.0, 0.0004, -0.0208), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
      mimic=("pinky1_joint", 1.0))

    return links, joints


def generate(side, flip, out_dir):
    mesh_dir = out_dir / "meshes"
    links, joints = finger_chain(side, flip)

    urdf = ['<?xml version="1.0"?>', f'<robot name="inmoov_{"left" if side == "l" else "right"}_hand">']
    urdf.append('  <link name="base_link" />')

    urdf.append(link_xml("forearm_link", f"{side}_forearm.stl", mesh_dir, "forearm"))
    urdf.append(joint_xml("forearm_mount_joint", "fixed", "base_link", "forearm_link",
                           (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))

    urdf.append(link_xml("hand_link", f"{side}_hand.stl", mesh_dir, "hand"))
    # wrist_roll_joint: real InMoov forearm-rotation joint, from inmoov.urdf.xacro.
    # Range is a half turn; mirrored sign per side to match the source's convention.
    if side == "r":
        wrist_lower, wrist_upper = -3.14159, 0.0
    else:
        wrist_lower, wrist_upper = 0.0, 3.14159
    urdf.append(joint_xml("wrist_joint", "revolute", "forearm_link", "hand_link",
                           (-0.0144, flip * 0.01, -0.2885), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                           lower=wrist_lower, upper=wrist_upper, velocity=WRIST_VELOCITY,
                           effort=WRIST_EFFORT))

    for name, mesh, mass_key in links:
        urdf.append(link_xml(name, mesh, mesh_dir, mass_key))
    for name, parent, child, xyz, rpy, axis, mimic in joints:
        # Driver joints (mimic is None) need real effort to actually respond to a
        # JointPositionController target - mimic joints stay effort=0 (follower-only,
        # they're constrained to the driver's position, not independently actuated).
        effort = FINGER_JOINT_EFFORT_MIMIC if mimic else FINGER_JOINT_EFFORT_DRIVER
        urdf.append(joint_xml(name, "revolute", parent, child, xyz, rpy, axis, mimic=mimic,
                               effort=effort))

    urdf.append("</robot>")

    out_path = out_dir / "hand.urdf"
    out_path.write_text("\n".join(urdf) + "\n")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    generate("l", -1.0, HERE / "left")
    generate("r", 1.0, HERE / "right")
