#!/usr/bin/env python3
"""Assembles sim/worlds/hand_world.sdf from left/hand.sdf + right/hand.sdf (which
generate_hand.py + `gz sdf -p` produce - see that script and CLAUDE.md's hand-model
provenance note for how those get built).

Run (from inside the robotics-gazebo container, or anywhere with `gz` on PATH):
    python3 generate_hand.py
    gz sdf -p left/hand.urdf > left/hand.sdf
    gz sdf -p right/hand.urdf > right/hand.sdf
    python3 build_world.py

Two fixups this script applies that the raw `gz sdf -p` output needs:
  - Mesh <uri> paths: hand.sdf's mesh URIs are relative to its own directory
    (sim/models/hand/{left,right}/), but hand_world.sdf lives in sim/worlds/ and
    embeds the model body directly (not via <include>), so they need a `../models/
    hand/{side}/` prefix to resolve from the world file's location.
  - Material: generate_hand.py's URDF has no <material> at all (a URDF-level
    <material><color rgba=.../></material> gets silently dropped by `gz sdf -p`
    anyway, converting to plain flat white regardless of the color specified - not
    worth carrying dead code for), so each visual converts with no material element.
    A low-ambient / mid-gray-diffuse material is inserted here post-conversion so the
    hands don't render as a flat, shading-less white silhouette.
"""

import re
from pathlib import Path

HERE = Path(__file__).parent
WORLD_PATH = HERE.parent.parent / "worlds" / "hand_world.sdf"

DRIVER_JOINTS = ["thumb1_joint", "index1_joint", "middle1_joint", "ring1_joint",
                 "pinky1_joint", "wrist_joint"]

HAND_MATERIAL = (
    "        <material>\n"
    "          <ambient>0.32 0.32 0.30 1</ambient>\n"
    "          <diffuse>0.80 0.80 0.76 1</diffuse>\n"
    "          <specular>0.25 0.25 0.25 1</specular>\n"
    "        </material>\n"
    "      </visual>"
)


def add_material(body):
    """generate_hand.py's URDF has no <material> at all (gz sdf -p silently drops a
    URDF-level <color> anyway - see generate_hand.py's docstring), so each <visual>
    converts with no material element rather than a wrong-but-present one. Insert
    one before each visual's closing tag."""
    return re.sub(r"\n(\s*)</visual>", "\n" + HAND_MATERIAL, body)


def extract_body(path):
    text = path.read_text()
    m = re.search(r"<model name='(\w+)'>\n(.*)\n  </model>", text, re.S)
    return m.group(2)


def controllers(model_name):
    parts = []
    for j in DRIVER_JOINTS:
        parts.append(f"""    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{j}</joint_name>
      <topic>/{model_name}/{j}/cmd_pos</topic>
      <p_gain>5</p_gain>
      <i_gain>0.05</i_gain>
      <d_gain>0.1</d_gain>
      <i_max>1</i_max>
      <i_min>-1</i_min>
      <cmd_max>5</cmd_max>
      <cmd_min>-5</cmd_min>
    </plugin>""")
    parts.append("""    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
    </plugin>""")
    return "\n".join(parts)


def model_block(name, body, x):
    return f"""    <model name='{name}'>
    <!-- Mesh authoring convention carried over from the old InMoov-derived model
         (hand hangs off an arm, fingers down at rest) - re-verified visually in
         the GUI (sim/docker/run.sh) after the mesh swap; adjust roll/pitch/yaw
         here if a future mesh source uses a different convention. -->
    <pose>{x} 0 1 3.14159 0 1.5708</pose>
{body}
    <joint name="world_mount" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint>

{controllers(name)}
  </model>"""


def build():
    left_body = extract_body(HERE / "left" / "hand.sdf")
    right_body = extract_body(HERE / "right" / "hand.sdf")

    left_body = left_body.replace("<uri>meshes/", "<uri>../models/hand/left/meshes/")
    right_body = right_body.replace("<uri>meshes/", "<uri>../models/hand/right/meshes/")

    left_body = add_material(left_body)
    right_body = add_material(right_body)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">

  <!-- Standalone Gazebo world for the dual InMoov-derived hand models (left + right),
       sourced from MyRobotLab/inmoov_ros meshes - see CLAUDE.md for full provenance.
       Both hands are fixed in space via a world-mount joint (not free-falling) so
       finger/wrist joints can be exercised directly through JointPositionController
       topics without also needing an arm/gravity-compensation setup.

       Physics engine is bullet-featherstone, NOT dartsim (the default used
       by box_world.sdf): dartsim in this Gazebo build silently ignores
       <mimic> joints ("physics engine does not support mimic constraints"),
       which breaks the 1-driven-joint-per-finger coupling this model
       depends on. Verified: bullet-featherstone applies mimic correctly - but
       ONLY when the driver joint is proximal (closer to hand_link) and every
       mimic joint is downstream of it. See generate_hand.py for the same note. -->
  <world name="hand_world">

    <physics name="1ms" type="bullet-featherstone">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <!-- Two hands ~0.4m apart, each ~10-30cm across; gz-sim's default camera pose
         frames a room-sized scene, so without this both hands are barely-visible
         specks on load. -->
    <gui>
      <camera name="user_camera">
        <pose>0.5 0.5 1.3 0 0.3 -2.35</pose>
      </camera>
    </gui>

    <light type="directional" name="sun">
      <!-- Shadows off: verified on the old single-hand model that the "jitter" isn't
           physics (sampled every joint at ~1kHz with zero commands sent, all flat to
           ~1e-8 rad) - default shadow-map settings are tuned for room-scale scenes and
           produce shadow acne/flicker on this hand's many close-set small meshes,
           which visually reads as jitter. Same root cause applies to this mesh set. -->
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{model_block("inmoov_left_hand", left_body, -0.2)}

{model_block("inmoov_right_hand", right_body, 0.2)}

  </world>
</sdf>
"""

    WORLD_PATH.write_text(world)
    print(f"wrote {WORLD_PATH}, {len(world)} bytes")


if __name__ == "__main__":
    build()
